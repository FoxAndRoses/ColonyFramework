using System;
using Sandbox.ModAPI;
using VRageMath;
using MyShipConnectorStatus = Sandbox.ModAPI.Ingame.MyShipConnectorStatus;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;

namespace ColonyFramework
{
    // Idle behavior: a drone with no mission no longer hovers forever wherever it stopped. It flies
    // home if far away, then either DOCKS at a free base connector and recharges to full, or — with
    // no connector free — LANDS 30-40 m from the base (deterministic per-drone spot so a fleet
    // spreads out), locks its gear, and POWER-NAPS: thrusters and gyros off, zero drain.
    // DroneUtil.PrepareForFlight (called by every mission launch path) wakes it cleanly.
    // Parking never hard-fails the asset — a failed attempt logs, cools down, and retries.
    public class ParkController
    {
        private const int PDecide = 0, PGoHome = 1, PDock = 2, PDescend = 3, PTouchdown = 4, PNapped = 5;

        private const double HomeRange       = 150.0; // farther than this from base: cruise home first
        private const double ArriveTol       = 10.0;
        private const double LandOffset      = 35.0;  // lateral distance of the landing spot from the base
        private const double DescendHover    = 15.0;  // hover point above the landing spot before touchdown
        private const double TouchdownSpeed  = 0.6;   // m/s gentle sink
        private const double TouchdownTimeout = 40.0; // grounded but no gear lock by then: nap anyway
        private const double LegTimeoutSecs  = 150.0;
        private const double RetryCooldownSecs = 60.0;
        private const float  HomeSpeed       = 30f;
        private const double CruiseAltitudeAgl = 100.0;

        private readonly NavState _nav = new NavState();
        private readonly BoreController _fly = new BoreController();
        private readonly OrbitNav _terrain = new OrbitNav();
        private readonly DockMachine _dock = new DockMachine();
        private readonly AvoidanceProbe _avoid = new AvoidanceProbe();

        private int _state = PDecide;
        private Vector3D _spot, _hoverPoint;
        private DateTime _legStart, _touchStart, _retryAt, _lastReroute, _lastLog;

        private ConnectorReservations _cons;

        public void Tick(Colony colony, AssetRecord asset, IMyCubeGrid grid, ConnectorReservations cons)
        {
            _cons = cons;
            if (DateTime.UtcNow < _retryAt) return; // cooling down after a failed attempt

            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            if (rc.IsUnderControl) return; // the player is flying it — never fight the pilot
            _nav.Refresh(grid, rc, DroneUtil.FindConnector(grid));
            var droneCon = DroneUtil.FindConnector(grid);
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (core == null) return;

            bool docked = droneCon != null && droneCon.Status == MyShipConnectorStatus.Connected;
            bool landed = DroneUtil.IsGearLocked(grid);

            if (_state == PNapped)
            {
                if (docked || landed) return;   // sleeping soundly
                _state = PDecide;               // knocked loose somehow — park again
            }
            if (docked) { Nap(grid, asset, true); return; }
            if (landed && _state != PTouchdown) { Nap(grid, asset, false); return; }

            switch (_state)
            {
                case PDecide:   Decide(colony, asset, grid, core, droneCon); break;
                case PGoHome:   TickGoHome(asset, grid, core); break;
                case PDock:     TickDock(asset, grid, core, droneCon); break;
                case PDescend:  TickDescend(asset, grid); break;
                case PTouchdown: TickTouchdown(asset, grid); break;
            }
        }

        private void Decide(Colony colony, AssetRecord asset, IMyCubeGrid grid, IMyCubeBlock core, IMyShipConnector droneCon)
        {
            Vector3D pos = grid.GetPosition();
            Vector3D basePos = core.GetPosition();
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;

            if (Vector3D.Distance(pos, basePos) > HomeRange)
            {
                DroneUtil.PrepareForFlight(grid);
                CruiseHome(grid, basePos + up * 100.0);
                _state = PGoHome;
                _legStart = DateTime.UtcNow;
                Log(asset, "idle far from base — heading home to park");
                return;
            }

            bool connectorAvailable = _cons != null
                ? _cons.AnyFree(core.CubeGrid, pos)
                : DroneUtil.FindFreeConnectorOnGroup(core.CubeGrid, pos) != null;
            if (droneCon != null && connectorAvailable)
            {
                DroneUtil.PrepareForFlight(grid);
                _dock.Reset();
                _state = PDock;
                Log(asset, "idle — docking at a free connector to recharge");
                return;
            }

            // No connector free: land at a deterministic per-drone spot away from the base pad.
            double ang = (asset.EntityId % 12) * (Math.PI / 6.0);
            Vector3D u = Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(up));
            Vector3D v = Vector3D.Normalize(Vector3D.Cross(up, u));
            Vector3D lateral = u * Math.Cos(ang) + v * Math.Sin(ang);
            _spot = _terrain.PinToSurface(basePos + lateral * LandOffset, up, 1.0);
            _hoverPoint = _spot + up * DescendHover;
            DroneUtil.PrepareForFlight(grid);
            FlyTo(grid, _hoverPoint, 8f, "landing hover");
            _state = PDescend;
            _legStart = DateTime.UtcNow;
            Log(asset, "idle, no free connector — landing near base");
        }

        private void TickGoHome(AssetRecord asset, IMyCubeGrid grid, IMyCubeBlock core)
        {
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            Vector3D standoff = core.GetPosition() + up * 100.0;
            double dist = Vector3D.Distance(grid.GetPosition(), standoff);
            if (dist > ArriveTol)
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                Vector3D via; string obstacle;
                if (_avoid.TryGetDetour(_nav, grid, standoff, out via, out obstacle)
                    && (DateTime.UtcNow - _lastReroute).TotalSeconds >= 3.0)
                { _lastReroute = DateTime.UtcNow; CruiseHome(grid, standoff, via); return; }
                if ((DateTime.UtcNow - _legStart).TotalSeconds > LegTimeoutSecs) GiveUp(asset, grid, "park go-home timeout");
                return;
            }
            var rc2 = DroneUtil.FindRc(grid);
            if (rc2 != null) rc2.SetAutoPilotEnabled(false);
            _state = PDecide; // at the base: re-evaluate (a connector may have freed up meanwhile)
        }

        private void TickDock(AssetRecord asset, IMyCubeGrid grid, IMyCubeBlock core, IMyShipConnector droneCon)
        {
            string r = _dock.Tick(grid, _nav, droneCon, core, _cons);
            if (r == DockMachine.Connected) { Nap(grid, asset, true); return; }
            if (r != null && r.StartsWith("fail:"))
            {
                // Docking didn't work out (connector taken, bump-fail...) — land instead next try.
                Log(asset, "park dock failed (" + r.Substring(5) + ") — will land instead");
                _fly.Release(grid);
                var rc = DroneUtil.FindRc(grid);
                if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
                _retryAt = DateTime.UtcNow.AddSeconds(RetryCooldownSecs);
                _state = PDecide;
            }
        }

        private void TickDescend(AssetRecord asset, IMyCubeGrid grid)
        {
            double dist = Vector3D.Distance(grid.GetPosition(), _hoverPoint);
            if (dist > 4.0 || _nav.Speed > 0.7)
            {
                if ((DateTime.UtcNow - _legStart).TotalSeconds > LegTimeoutSecs) GiveUp(asset, grid, "park descend timeout");
                return;
            }
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
            _state = PTouchdown;
            _touchStart = DateTime.UtcNow;
        }

        private void TickTouchdown(AssetRecord asset, IMyCubeGrid grid)
        {
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            // Gentle sink onto the spot; gears grab as soon as they touch.
            _fly.ThrustAlong(grid, -up, TouchdownSpeed, 0.6f);
            if (DroneUtil.LockGear(grid))
            {
                _fly.Release(grid);
                Nap(grid, asset, false);
                return;
            }
            if ((DateTime.UtcNow - _touchStart).TotalSeconds > TouchdownTimeout)
            {
                // Grounded (or close enough) but the gear never gripped — stop pushing, rest here.
                _fly.Release(grid);
                Nap(grid, asset, false);
            }
        }

        // Power-nap: nothing draws power while parked. PrepareForFlight undoes all of this on dispatch.
        private void Nap(IMyCubeGrid grid, AssetRecord asset, bool atConnector)
        {
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
            DroneUtil.SetBatteriesRecharge(grid, atConnector); // topping up while docked; auto when landed
            DroneUtil.SetThrustersAndGyros(grid, false);
            _state = PNapped;
            Log(asset, atConnector ? "parked at connector, recharging (powered down)" : "parked on the ground (powered down)");
        }

        // Parking must never strand or hard-fail an asset: stabilize, log, cool down, try again later.
        private void GiveUp(AssetRecord asset, IMyCubeGrid grid, string reason)
        {
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
            _retryAt = DateTime.UtcNow.AddSeconds(RetryCooldownSecs);
            _state = PDecide;
            Log(asset, "parking attempt failed (" + reason + "), retrying in " + (int)RetryCooldownSecs + "s");
        }

        private void CruiseHome(IMyCubeGrid grid, Vector3D target, Vector3D? via = null)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true;
            rc.ClearWaypoints();
            Vector3D pos = grid.GetPosition();
            double agl;
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            if (DroneUtil.TryGetAltitude(grid, out agl) && agl < CruiseAltitudeAgl)
                rc.AddWaypoint(pos + up * (CruiseAltitudeAgl - agl), "climb to cruise");
            if (via.HasValue) rc.AddWaypoint(via.Value, "avoid detour");
            rc.AddWaypoint(target, "park: base standoff");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = HomeSpeed;
            rc.SetAutoPilotEnabled(true);
        }

        private void FlyTo(IMyCubeGrid grid, Vector3D target, float speed, string label)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true;
            rc.ClearWaypoints();
            rc.AddWaypoint(target, label);
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = speed;
            rc.SetAutoPilotEnabled(true);
        }

        private void Log(AssetRecord asset, string msg)
        {
            if ((DateTime.UtcNow - _lastLog).TotalSeconds < 5.0) return;
            _lastLog = DateTime.UtcNow;
            VRage.Utils.MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] park '{0}': {1}", asset.Name, msg));
        }
    }
}
