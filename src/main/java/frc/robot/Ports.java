package frc.robot;

// Single code location to lay out all ports and buses, to ensure no conflicts
public enum Ports {
	ALGAE_DEPLOY(8, Bus.MECH), //Algae Pivot
	ALGAE_ROLLERS(9, Bus.MECH), //Algae Intake
	CORAL_DEPLOY(10, Bus.MECH), //Coral Intake 4bar
	CORAL_ROLLERS(11, Bus.MECH), //Coral Intake
	INDEXER(12, Bus.MECH), //Indexer
	EE_PIECE(13, Bus.RIO), //End Effector
	EE_POSITION(16, Bus.RIO), //End Effector Pivot
	EE_CANDI(0, Bus.RIO), //End Effector CANdi
	ELEVATOR_MAIN(14, Bus.MECH), //Elevator Inside
	ELEVATOR_FOLLOWER(15, Bus.MECH), //Elevator Outside
	CLIMBER_DEPLOY(17, Bus.MECH), //Climber Pivot
	CLIMBER_ROLLERS(18, Bus.MECH), //Climber Intake
	CANDLE(21, Bus.MECH),
	ENCODER_41T(4, Bus.RIO), // Top End Effector (direct)
	ENCODER_40T(5, Bus.RIO), // Bottom End Effector (geared)
	INDEXER_BEAMBREAK(8, null); // Digital Input

    public enum Bus {
        RIO("rio"),
        MECH("LynkMechanisms"),
        SWERVE("LynkSwerve");
    
        public final String name;

        private Bus(String name) {
            this.name = name;
        }
    }    

	public final int id;
	public final Bus bus;

	private Ports(int id, Bus bus) {
		this.id = id;
		this.bus = bus;
	}
}