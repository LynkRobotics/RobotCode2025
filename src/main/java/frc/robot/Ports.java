package frc.robot;

// Single code location to lay out all ports and buses, to ensure no conflicts
public enum Ports {
	ALGAE_DEPLOY(8, Bus.MECH),
	ALGAE_ROLLERS(9, Bus.MECH),
	CORAL_DEPLOY(10, Bus.MECH),
	CORAL_ROLLERS(11, Bus.MECH),
	INDEXER(12, Bus.MECH),
	EE_PIECE(13, Bus.RIO),
	EE_POSITION(16, Bus.RIO),
	ELEVATOR_MAIN(14, Bus.MECH),
	ELEVATOR_FOLLOWER(15, Bus.MECH),
	CLIMBER_DEPLOY(17, Bus.MECH),
	CLIMBER_ROLLERS(18, Bus.MECH),
	CANDLE(21, Bus.MECH);

    public enum Bus {
        RIO("rio"),
        MECH("LynkMechanisms"),
        SWERVE("LynkSwerve");
    
        public final String name;

        private Bus(String name) {
            this.name = name;
        }
    }    

	// EE_CORAL_BREAMBREAK(1, "RioDigitalIn"),
	// EE_ALGAE_BEAMBREAK(0, "RioDigitalIn"),
	// INDEXER_BEAMBREAK(8, "RioDigitalIn"),

	// ENCODER_41T(4, "canivore1"),
	// ENCODER_39T(5, "canivore1"),

	// PHYSICAL_BUTTON(9, "RioDigitalIn");

	public final int id;
	public final Bus bus;

	private Ports(int id, Bus bus) {
		this.id = id;
		this.bus = bus;
	}
}