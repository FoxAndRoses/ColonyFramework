using System.Collections.Generic;
using Sandbox.ModAPI;
using VRageMath;
using VRage.Game.ModAPI;
using MyPlanetElevation = Sandbox.ModAPI.Ingame.MyPlanetElevation;
using MyShipConnectorStatus = Sandbox.ModAPI.Ingame.MyShipConnectorStatus;
using IMyCubeGrid  = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;
using IMySlimBlock = VRage.Game.ModAPI.IMySlimBlock;
using IMyInventory = VRage.Game.ModAPI.IMyInventory;

namespace ColonyFramework
{
    // Stateless grid/power helpers shared by the drone execution services.
    // No per-mission state lives here — callers own that.
    public static class DroneUtil
    {
        public static IMyRemoteControl FindRc(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return null;
            var rcs = new List<IMyRemoteControl>();
            ts.GetBlocksOfType(rcs);
            return rcs.Count > 0 ? rcs[0] : null;
        }

        public static List<IMyShipDrill> FindDrills(IMyCubeGrid grid)
        {
            var drills = new List<IMyShipDrill>();
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts != null) ts.GetBlocksOfType(drills);
            return drills;
        }

        // First connector on the grid (the drone's docking connector).
        public static IMyShipConnector FindConnector(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return null;
            var cons = new List<IMyShipConnector>();
            ts.GetBlocksOfType(cons);
            return cons.Count > 0 ? cons[0] : null;
        }

        // Nearest free (not Connected) connector across the physical grid group of 'anyGridInGroup' —
        // used to find an open base connector on the colony-core's structure.
        public static IMyShipConnector FindFreeConnectorOnGroup(IMyCubeGrid anyGridInGroup, Vector3D nearTo)
        {
            var grids = new List<IMyCubeGrid>();
            var group = anyGridInGroup.GetGridGroup(GridLinkTypeEnum.Physical);
            if (group != null) group.GetGrids(grids);
            if (grids.Count == 0) grids.Add(anyGridInGroup);

            var cons = new List<IMyShipConnector>();
            IMyShipConnector best = null;
            double bestSq = double.MaxValue;
            for (int g = 0; g < grids.Count; g++)
            {
                var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grids[g]);
                if (ts == null) continue;
                cons.Clear();
                ts.GetBlocksOfType(cons);
                for (int i = 0; i < cons.Count; i++)
                {
                    if (cons[i].Status == MyShipConnectorStatus.Connected) continue;
                    double sq = Vector3D.DistanceSquared(cons[i].GetPosition(), nearTo);
                    if (sq < bestSq) { bestSq = sq; best = cons[i]; }
                }
            }
            return best;
        }

        public static bool TryGetAltitude(IMyCubeGrid grid, out double altitude)
        {
            altitude = 0;
            var rc = FindRc(grid);
            if (rc == null) return false;
            return rc.TryGetPlanetElevation(MyPlanetElevation.Surface, out altitude);
        }

        // Fuel/consumable inventories are NOT cargo: a reactor's uranium, gas tanks, and O2/H2
        // generators hold fuel that mined ore never goes into and that can't be unloaded — counting
        // them makes the drone read "not empty" forever (the "cargo transfer incomplete" lie).
        private static bool IsCargoBlock(IMyCubeBlock fat)
        {
            if (fat is IMyReactor) return false;
            if (fat is IMyGasTank) return false;
            if (fat is IMyGasGenerator) return false;
            return true;
        }

        public static double CargoFill(IMyCubeGrid grid)
        {
            var blocks = new List<IMySlimBlock>();
            grid.GetBlocks(blocks);
            double cur = 0, max = 0;
            for (int b = 0; b < blocks.Count; b++)
            {
                var fat = blocks[b].FatBlock as IMyCubeBlock;
                if (fat == null || !fat.HasInventory || !IsCargoBlock(fat)) continue;
                for (int i = 0; i < fat.InventoryCount; i++)
                {
                    var inv = fat.GetInventory(i);
                    if (inv == null) continue;
                    cur += (double)inv.CurrentVolume;
                    max += (double)inv.MaxVolume;
                }
            }
            return max > 0 ? cur / max : 0;
        }

        public static bool HasInfinitePower(IMyCubeGrid grid)
        {
            var blocks = new List<IMySlimBlock>();
            grid.GetBlocks(blocks);
            for (int b = 0; b < blocks.Count; b++)
            {
                var reactor = blocks[b].FatBlock as IMyReactor;
                if (reactor != null && reactor.IsWorking) return true;
            }
            return false;
        }

        public static double MinBatteryCharge(IMyCubeGrid grid)
        {
            var blocks = new List<IMySlimBlock>();
            grid.GetBlocks(blocks);
            double min = 1.0;
            bool found = false;
            for (int b = 0; b < blocks.Count; b++)
            {
                var bat = blocks[b].FatBlock as IMyBatteryBlock;
                if (bat == null) continue;
                double charge = bat.MaxStoredPower > 0
                    ? (double)bat.CurrentStoredPower / (double)bat.MaxStoredPower : 1.0;
                if (charge < min) min = charge;
                found = true;
            }
            return found ? min : 1.0; // no batteries = external power, don't recall
        }

        // Sums battery stored energy (MWh) and current output (MW); returns battery count.
        public static int SumBatteryPower(IMyCubeGrid grid, out double stored, out double output)
        {
            stored = 0; output = 0;
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return 0;
            var bats = new List<IMyBatteryBlock>();
            ts.GetBlocksOfType(bats);
            for (int i = 0; i < bats.Count; i++)
            {
                stored += bats[i].CurrentStoredPower;
                output += bats[i].CurrentOutput;
            }
            return bats.Count;
        }

        // Snapshot for the commissioning power self-test (taken with all consumers spiked via SetSpike):
        // battery energy (stored / capacity, MWh) and the supply split (reactor vs battery output, MW).
        // Under full load the battery's output is the deficit the reactor can't cover — i.e. how fast the
        // battery would drain in a mission, which sizes how much we recharge before the next dispatch.
        public static void MeasurePower(IMyCubeGrid grid, out double batteryStored, out double batteryMax,
                                        out double reactorOutput, out double batteryOutput)
        {
            batteryStored = batteryMax = reactorOutput = batteryOutput = 0;
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var bats = new List<IMyBatteryBlock>();
            ts.GetBlocksOfType(bats);
            for (int i = 0; i < bats.Count; i++)
            {
                batteryStored += bats[i].CurrentStoredPower;
                batteryMax    += bats[i].MaxStoredPower;
                batteryOutput += bats[i].CurrentOutput;
            }
            var reactors = new List<IMyReactor>();
            ts.GetBlocksOfType(reactors);
            for (int i = 0; i < reactors.Count; i++) reactorOutput += reactors[i].CurrentOutput;
        }

        // Spike (on) or clear (off) all thrusters + drills — used for the commissioning power draw test.
        public static void SetSpike(IMyCubeGrid grid, bool on)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++) thrusters[i].ThrustOverridePercentage = on ? 1f : 0f;
            var drills = new List<IMyShipDrill>();
            ts.GetBlocksOfType(drills);
            for (int i = 0; i < drills.Count; i++) drills[i].Enabled = on;
        }

        public static void SetDrills(IMyCubeGrid grid, bool on)
        {
            var drills = FindDrills(grid);
            for (int i = 0; i < drills.Count; i++) drills[i].Enabled = on;
        }

        // Force all batteries to Recharge (pull power from the docked base) or back to Auto for flight.
        public static void SetBatteriesRecharge(IMyCubeGrid grid, bool recharge)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var bats = new List<IMyBatteryBlock>();
            ts.GetBlocksOfType(bats);
            for (int i = 0; i < bats.Count; i++)
                bats[i].ChargeMode = recharge
                    ? Sandbox.ModAPI.Ingame.ChargeMode.Recharge
                    : Sandbox.ModAPI.Ingame.ChargeMode.Auto;
        }

        // Push all items from the drone's inventories into the base (through the locked connector's
        // conveyor network) — one pass per call. Returns true once the drone holds nothing.
        public static bool UnloadCargo(IMyCubeGrid drone, IMyShipConnector droneCon)
        {
            var baseCon = droneCon != null ? droneCon.OtherConnector : null;
            if (baseCon == null) return false; // not actually connected — don't claim done

            var dsts = new List<IMyInventory>();
            GatherInventories(baseCon.CubeGrid, dsts, drone, true); // whole base group, excluding the drone
            if (dsts.Count == 0) return false; // nowhere to put it

            var srcs = new List<IMyInventory>();
            GatherInventories(drone, srcs, null, false); // drone grid only

            double remaining = 0;
            for (int s = 0; s < srcs.Count; s++)
            {
                var src = srcs[s];
                bool moved = true; int guard = 0;
                while (src.ItemCount > 0 && moved && guard++ < 500)
                {
                    moved = false;
                    for (int d = 0; d < dsts.Count; d++)
                        if (src.TransferItemTo(dsts[d], 0)) { moved = true; break; } // move item 0; next shifts down
                }
                remaining += (double)src.CurrentVolume;
            }
            return remaining < 1e-6;
        }

        // Collect every block inventory on a grid (or its whole physical group), optionally skipping
        // one grid (e.g. the drone itself once it has merged into the base group via the connector).
        private static void GatherInventories(IMyCubeGrid anyGrid, List<IMyInventory> into, IMyCubeGrid exclude, bool wholeGroup)
        {
            var grids = new List<IMyCubeGrid>();
            if (wholeGroup)
            {
                var group = anyGrid.GetGridGroup(GridLinkTypeEnum.Physical);
                if (group != null) group.GetGrids(grids);
            }
            if (grids.Count == 0) grids.Add(anyGrid);

            var blocks = new List<IMySlimBlock>();
            for (int g = 0; g < grids.Count; g++)
            {
                if (grids[g] == exclude) continue;
                blocks.Clear();
                grids[g].GetBlocks(blocks);
                for (int b = 0; b < blocks.Count; b++)
                {
                    var fat = blocks[b].FatBlock as IMyCubeBlock;
                    if (fat == null || !fat.HasInventory || !IsCargoBlock(fat)) continue; // skip reactor/fuel
                    for (int i = 0; i < fat.InventoryCount; i++)
                    {
                        var inv = fat.GetInventory(i);
                        if (inv != null) into.Add(inv);
                    }
                }
            }
        }

        // Unlock landing gear and disconnect connectors so the drone can launch.
        public static void ReleaseGrid(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;

            var gears = new List<SpaceEngineers.Game.ModAPI.IMyLandingGear>();
            ts.GetBlocksOfType(gears);
            for (int i = 0; i < gears.Count; i++)
            {
                gears[i].AutoLock = false;
                gears[i].Unlock();
            }

            var connectors = new List<IMyShipConnector>();
            ts.GetBlocksOfType(connectors);
            for (int i = 0; i < connectors.Count; i++) connectors[i].Disconnect();
        }
    }
}
