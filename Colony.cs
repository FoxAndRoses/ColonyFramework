namespace ColonyFramework
{
    public class Colony
    {
        public ColonyState State { get; private set; }
        public DepositManager Deposits { get; private set; }
        public MissionManager Missions { get; private set; }
        public AssetManager Assets { get; private set; }
        public ResourceSnapshot Resources { get; private set; }

        public Colony(ColonyState state)
        {
            State = state;
            Deposits = new DepositManager(state);
            Missions = new MissionManager(state, Deposits);
            Assets = new AssetManager(state);
            Resources = new ResourceSnapshot();
        }

        public long OwnerKey { get { return State.OwnerKey; } }
        public bool Active { get { return State.Active; } set { State.Active = value; } }
    }
}
