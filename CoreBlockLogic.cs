using Sandbox.Common.ObjectBuilders;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.ObjectBuilders;
using VRage.Utils;

namespace ColonyFramework
{
    [MyEntityComponentDescriptor(typeof(MyObjectBuilder_MyProgrammableBlock), false, "ColonyCore")]
    public class CoreBlockLogic : MyGameLogicComponent
    {
        private IMyCubeBlock _block;

        public override void Init(MyObjectBuilder_EntityBase objectBuilder)
        {
            _block = Entity as IMyCubeBlock;
            if (CommandCoreSession.Instance != null && _block != null)
                CommandCoreSession.Instance.RegisterCore(_block);
            MyLog.Default.WriteLineAndConsole("[ColonyFramework] Colony Core initialized: " + Entity.EntityId);
        }

        public override void Close()
        {
            if (CommandCoreSession.Instance != null && _block != null)
                CommandCoreSession.Instance.UnregisterCore(_block);
            MyLog.Default.WriteLineAndConsole("[ColonyFramework] Colony Core removed: " + Entity.EntityId);
        }
    }
}
