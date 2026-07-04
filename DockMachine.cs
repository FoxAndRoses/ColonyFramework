using System;
using Sandbox.ModAPI;
using VRageMath;
using MyShipConnectorStatus = Sandbox.ModAPI.Ingame.MyShipConnectorStatus;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;

namespace ColonyFramework
{
    // Reusable connector-docking machine — the PROVEN approach/shimmy/align/reverse sequence
    // (autopilot diagonals + dampeners + centred magnet reverse), extracted from the welder's copy
    // so any controller can dock a drone at a free base connector (welder resupply, idle parking).
    // The miner keeps its own battle-tested dock untouched. Tick() returns null while working,
    // DockMachine.Connected on lock, or "fail:<reason>" for the caller's retry/fallback logic.
    public class DockMachine
    {
        public const string Connected = "connected";

        private const int SubApproach = 0, SubShimmy = 1, SubAlign = 2, SubReverse = 3;

        // Constants proven by the miner's dock.
        private const double StageFwd = 20.0, ShimmyTop = 25.0, ShimmyDrop = 5.0, ShimmyStep = 8.0, DockClearance = 1.0;
        private const double MoveSpeed = 6.0, ArriveTol = 3.0, AlignDot = 0.98, SettleSpeed = 0.5;
        private const double ReverseSpeed = 1.0, ReverseOvershoot = 5.0, BumpDist = 3.0, BumpFailSecs = 30.0, LockTrySecs = 1.5;
        private const double LateralTol = 0.15, CenterEnter = 0.30, CenterGain = 0.6, CenterMinSpeed = 0.05;
        private const double CrawlNear = 0.25, CrawlMid = 1.0, CrawlFar = 2.0, CrawlMidDist = 5.0, CrawlFarDist = 10.0;
        private const double ContactSpeedEps = 0.1;
        private const double TimeoutSecs = 180.0;

        private readonly BoreController _fly = new BoreController();
        private long _baseConId;
        private int _sub;
        private double _shimmyHeight; private bool _shimmyOut;
        private bool _centering;
        private double _minDist = double.MaxValue;
        private DateTime _start, _bumpStart, _lastLockTry;

        public void Reset()
        {
            _baseConId = 0;
            _sub = SubApproach;
            _centering = false;
            _minDist = double.MaxValue;
            _start = DateTime.UtcNow;
            _bumpStart = default(DateTime);
        }

        public string Tick(IMyCubeGrid grid, NavState nav, IMyShipConnector droneCon, IMyCubeBlock core)
        {
            if (droneCon == null || core == null) return "fail:connector/core lost";
            if (droneCon.Status == MyShipConnectorStatus.Connected) { _fly.Release(grid); return Connected; }
            if ((DateTime.UtcNow - _start).TotalSeconds > TimeoutSecs) return "fail:dock timeout";

            var baseCon = MyAPIGateway.Entities.GetEntityById(_baseConId) as IMyShipConnector;
            if (baseCon == null)
            {
                baseCon = DroneUtil.FindFreeConnectorOnGroup(core.CubeGrid, grid.GetPosition());
                if (baseCon == null) return "fail:no free base connector";
                _baseConId = baseCon.EntityId;
                Vector3D up0 = nav.Valid ? nav.GravityUp : Vector3D.Up;
                _shimmyHeight = ShimmyTop;
                _shimmyOut = true;
                FlyTo(grid, baseCon.GetPosition() + baseCon.WorldMatrix.Forward * StageFwd + up0 * ShimmyTop, "dock approach");
                _sub = SubApproach;
                return null;
            }

            Vector3D bPos = baseCon.GetPosition();
            Vector3D bFwd = baseCon.WorldMatrix.Forward;
            Vector3D dPos = droneCon.GetPosition();
            Vector3D dFwd = droneCon.WorldMatrix.Forward;
            Vector3D up = nav.Valid ? nav.GravityUp : Vector3D.Up;
            Vector3D rcPos = grid.GetPosition();

            if (_sub == SubApproach)
            {
                Vector3D top = bPos + bFwd * StageFwd + up * ShimmyTop;
                if (Vector3D.Distance(rcPos, top) <= ArriveTol && nav.Speed < SettleSpeed)
                {
                    _shimmyHeight = ShimmyTop - ShimmyDrop;
                    _shimmyOut = true;
                    FlyTo(grid, ShimmyPoint(bPos, bFwd, up), "shimmy");
                    _sub = SubShimmy;
                }
            }
            else if (_sub == SubShimmy)
            {
                Vector3D target = ShimmyPoint(bPos, bFwd, up);
                if (Vector3D.Distance(rcPos, target) <= ArriveTol && nav.Speed < SettleSpeed)
                {
                    if (_shimmyHeight <= DockClearance + 0.1)
                    {
                        var rc = DroneUtil.FindRc(grid);
                        if (rc != null) rc.SetAutoPilotEnabled(false);
                        _sub = SubAlign;
                    }
                    else
                    {
                        _shimmyHeight = Math.Max(DockClearance, _shimmyHeight - ShimmyDrop);
                        _shimmyOut = !_shimmyOut;
                        FlyTo(grid, ShimmyPoint(bPos, bFwd, up), "shimmy");
                    }
                }
            }
            else if (_sub == SubAlign)
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                double a = _fly.Face(grid, dFwd, -bFwd);
                if (a > AlignDot && nav.Speed < SettleSpeed)
                {
                    _sub = SubReverse;
                    _bumpStart = default(DateTime);
                    _centering = false;
                    _minDist = double.MaxValue;
                }
            }
            else // SubReverse
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                if (droneCon.Status == MyShipConnectorStatus.Connectable
                    && (DateTime.UtcNow - _lastLockTry).TotalSeconds > LockTrySecs)
                {
                    _lastLockTry = DateTime.UtcNow;
                    droneCon.Connect();
                    if (droneCon.Status == MyShipConnectorStatus.Connected) { _fly.Release(grid); return Connected; }
                }
                _fly.Face(grid, dFwd, -bFwd);

                Vector3D rel = dPos - bPos;
                double along = Vector3D.Dot(rel, bFwd);
                Vector3D offAxis = rel - bFwd * along;
                double lateral = offAxis.Length();
                double dist = rel.Length();
                double inwardSpeed = Vector3D.Dot(nav.Velocity, -bFwd);

                if (_centering) { if (lateral <= LateralTol) _centering = false; }
                else if (lateral > CenterEnter) _centering = true;

                if (_centering)
                {
                    double shiftSpeed = Math.Min(CrawlNear, Math.Max(CenterMinSpeed, lateral * CenterGain));
                    _fly.ThrustAlong(grid, -offAxis / lateral, shiftSpeed, 1.0f, 0.5f);
                    _bumpStart = default(DateTime);
                }
                else
                {
                    double rs = dist > CrawlFarDist ? CrawlFar : dist > CrawlMidDist ? CrawlMid : CrawlNear;
                    _fly.ThrustAlong(grid, -bFwd, rs, 1.0f);
                    if (dist <= BumpDist && inwardSpeed < ContactSpeedEps)
                    {
                        if (_bumpStart == default(DateTime)) _bumpStart = DateTime.UtcNow;
                        else if ((DateTime.UtcNow - _bumpStart).TotalSeconds > BumpFailSecs)
                            return "fail:dock bump failed";
                    }
                    else _bumpStart = default(DateTime);
                }
                if (dist < _minDist) _minDist = dist;
                if (dist > _minDist + ReverseOvershoot) return "fail:dock reverse overshoot";
            }
            return null;
        }

        private Vector3D ShimmyPoint(Vector3D bPos, Vector3D bFwd, Vector3D up)
        {
            return bPos + bFwd * (StageFwd + (_shimmyOut ? ShimmyStep : 0.0)) + up * _shimmyHeight;
        }

        private void FlyTo(IMyCubeGrid grid, Vector3D target, string label)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true;
            rc.ClearWaypoints();
            rc.AddWaypoint(target, label);
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = (float)MoveSpeed;
            rc.SetAutoPilotEnabled(true);
        }
    }
}
