package us.ihmc.moonwalking.models.SeriesElasticActuator;

public class SEAParameters
{
    private static final SEAParameters sEAParameters;
    
    static
    {
	sEAParameters = new SEAParameters();
    }
    
    public static final double GEAR_RATIO = 160.0;
    
    //100202 pdn: Nic says 15A is ok.
    public static final double MAXIMUM_AMPLIFIER_CURRENT = 30.0; //25.0; //15.0; // Amps
    public static final double CONTINUOUS_AMPLIFIER_CURRENT = 15.0;
    // Maximum motor torque li`mits.
    //For the 85x13 Robodrive motor
    public static final double PEAK_MOTOR_TORQUE = 1e6; //4.5; // Nm at the motor
							// output. Note that
							// needs to be geared up
							// for gearbox output
    public static final double RPM_2_RAD_PER_SEC = 2.0 * Math.PI / 60.0;
    
    //4800 for size 32 CSD
    public static final double MAXIMUM_GEARBOX_OUTPUT_SPEED = 4800.0 * RPM_2_RAD_PER_SEC / GEAR_RATIO;
    public static final double TORQUE_SENSITIVITY = 0.13; // Nm/amp
    public static final double PEAK_MOTOR_TORQUE_FROM_AMPLIFIER_LIMIT = MAXIMUM_AMPLIFIER_CURRENT * TORQUE_SENSITIVITY;
    public static final double PEAK_DRIVE_TORQUE = Math.min(PEAK_MOTOR_TORQUE, PEAK_MOTOR_TORQUE_FROM_AMPLIFIER_LIMIT) * GEAR_RATIO;

    static
    {
	System.out.println("MAXIMUM_GEARBOX_OUTPUT_SPEED = " + MAXIMUM_GEARBOX_OUTPUT_SPEED);
    }

    
    static
    {
	System.out.println("PEAK_DRIVE_TORQUE = " + PEAK_DRIVE_TORQUE);
    }

    // Use CSD size 20. Note, the catalog inertia at at the input. multiply by
    // the GR to get the output
    public static final double GEARBOX_OUTPUT_MASS = 0.1;

    // Just make this small
    // public final double SPRING_MASS = 0.01;
    // public final double SPRING_MASS_EFFECTIVE_RADIUS = 0.01;
    // public final double SPRING_OUTPUT_I = 0.5 * SPRING_MASS *
    // SPRING_MASS_EFFECTIVE_RADIUS * SPRING_MASS_EFFECTIVE_RADIUS;

    // public final double LOAD_MASS = 1000000000.0; //Clamp the actuator//3.0;
    public static double LOAD_MASS = 15.0; // Unclamped load
    public static final double LOAD_MASS_EFFECTIVE_RADIUS = 0.5; // Leg is 1.0
								 // long, with
								 // COM halfway
								 // down

    // For graphical purposes only
    public static final double MOTOR_BODY_RADIUS = 0.1;
    public static final double MOTOR_ROTOR_I = 1.27e-4;

    // Mass properties
    // Use Moog BN42-23AF-02 for initial guess
    //Motor I = 0.61 kgcm^2
    
    public static final double MOTOR_ROTOR_MASS = 0.61 /(100.0 * 100.0); // Assume that rotor is
							 // 20% of motor
    public static final double MOTOR_ROTOR_MASS_EFFECTIVE_RADIUS = 0.01;
    
    
    public static final double MOTOR_SHAFT_RADIUS = MOTOR_BODY_RADIUS / 10.0;
    public static double LOAD_INERTIA;
    
    public static void recalculateLoadInteria()
    {
	LOAD_INERTIA = LOAD_MASS * LOAD_MASS_EFFECTIVE_RADIUS * LOAD_MASS_EFFECTIVE_RADIUS;
    }
    
    static
    {
	recalculateLoadInteria();
    }

    // public final double GEARBOX_OUTPUT_MASS_EFFECTIVE_RADIUS = 0.01;
    public static final double GEARBOX_OUTPUT_I = 1.09e-4 * GEAR_RATIO * GEAR_RATIO; // 0.5
										     // *
										     // GEARBOX_OUTPUT_MASS
										     // *
										     // GEARBOX_OUTPUT_MASS_EFFECTIVE_RADIUS
										     // *
										     // GEARBOX_OUTPUT_MASS_EFFECTIVE_RADIUS;

    // Encoder Resolutions
    public static final double APS_ENCODER_COUNTS_PER_RADIAN = 1.0 / (2.75e-3 * (Math.PI / 180.0)); // (20,845 counts/rad)
//    public static final double LOW_RESOLUTION_ENCODER = 1e3;

    public static final double springConstantDefault = 6500.0; // Use a
								     // Units of
								     // Nm/rad
    public static final double kpGearDefault = 61000.0; //110000.0; //[Nm/rad] This is the minimum
							 // measured value of
							 // the harmonic drive
    public static final double kdGearDefault = 10000.0; //100.0; // Not sure how this
						       // related to critically
						       // damped.
    public static final double gearboxLinearDampingDefault = 0.0;

}
