package us.ihmc.moonwalking.models.SeriesElasticActuator;


import com.yobotics.simulationconstructionset.*;
import com.yobotics.simulationconstructionset.gui.BodePlotConstructor;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;

import us.ihmc.utilities.linearDynamicSystems.TransferFunction;


public class ActuatorController implements RobotController
{
//    public boolean DO_GEARBOX_SPEED_LIMIT = true; // true;
    public static final boolean USE_MOTOR = true;
    
    private static int instanceCounter = 0;
    private final String suffix = "_" + instanceCounter;

    private final YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("actuatorController" + suffix);
    private final YoVariable wn = new YoVariable("wn" + suffix, "Natural Freq gear position from desired torque of system under closed loop torque control",
	    yoVariableRegistry);

    // Constants, or inputs from a higher level.
    private final YoVariable kpGear = new YoVariable("kpGear" + suffix, "k_R: Stiffness between motor and gear train at the input. Units are Nm/rad",
	    yoVariableRegistry);
    private final YoVariable kdGear = new YoVariable("kdGear" + suffix, "b_R: Damping gain between motor and gear train at the input. Units are Nm/(rad/sec)",
	    yoVariableRegistry);
    private final YoVariable zeta = new YoVariable("zeta" + suffix,
	    "Damping ratio gear position from desired torque of system under closed loop torque control", yoVariableRegistry);
    private final YoVariable totalGearboxOutputTorque = new YoVariable("totalGearboxOutputTorque" + suffix,
	    "tau_2: The torque applied to the gearbox output joint", yoVariableRegistry);
    private final YoVariable tauGearboxDamping = new YoVariable("tauGearboxDamping" + suffix,
	    "tau_damp: Friction in the gearbox due to velocity limit or grease", yoVariableRegistry);
    private final YoVariable tauGearboxConstraint = new YoVariable("tauGearboxConstraint" + suffix,
	    "tau_R: The torque required to enforce the gearbox constraint", yoVariableRegistry);
    private final YoVariable tauErrorDot = new YoVariable("tauErrorDot" + suffix, "Rate of change of (Desired Torque - Actual Torque)", yoVariableRegistry);
    private final YoVariable tauError = new YoVariable("tauError" + suffix, "Desired Torque - Actual Torque", yoVariableRegistry);

    
    
    // private final YoVariable tauDesiredDot = new YoVariable("tauDesiredDot",
    // yoVariableRegistry);
    private final YoVariable tauDrive = new YoVariable("tauDrive" + suffix, "The desired drive torque based on feedforward and feedback", yoVariableRegistry);
    private final YoVariable tauDriveAfterLosses = new YoVariable("tauDriveAfterLosses" + suffix, "The actual drive torque after gear box losses", yoVariableRegistry);
    private final YoVariable gearboxEfficiency = new YoVariable("gearboxEfficiency" + suffix, "This is the efficiency due to " +
    		"losses in grease and friction", yoVariableRegistry); 
    
//    private final YoVariable motorCurrent = new YoVariable("motorCurrent" + suffix, "The current to the motor", yoVariableRegistry);
    
    // private final YoVariable tauDist = new YoVariable("tauDist",
    // yoVariableRegistry);

    // Computed variables.
    private final YoVariable tauDesired = new YoVariable("tauDesired" + suffix, "Desired torque on the load provided by the SEA spring", yoVariableRegistry);
    private final YoVariable tauActualDot = new YoVariable("tauActualDot" + suffix, "Actual torque rate of change as measured by the SEA, Units Nm/s",
	    yoVariableRegistry);
    private final YoVariable tauActual = new YoVariable("tauActual" + suffix, "Actual torque as measured by the SEA, Units Nm", yoVariableRegistry);

    private final YoVariable springDeflection = new YoVariable("springDeflection" + suffix, yoVariableRegistry);
    private final YoVariable springDeflectionDot = new YoVariable("springDeflectionDot" + suffix, yoVariableRegistry);
    private final YoVariable springDeflectionEncoder = new YoVariable("springDeflectionEncoder" + suffix, yoVariableRegistry);
    private final YoVariable springDeflectionEncoderDot = new YoVariable("springDeflectionEncoderDot" + suffix, yoVariableRegistry);

    // private final YoVariable springDeflectionDotEncoder = new
    // YoVariable("springDeflectionDotEncoder" + suffix, yoVariableRegistry);

    private final YoVariable springDamping = new YoVariable("springDamping" + suffix,
	    "Spring damping of the Series Elastic Actuator spring. Units are Nm/(rad/s)", yoVariableRegistry);
    private final YoVariable springConstant = new YoVariable("springConstant" + suffix,
	    "Spring constant of the Series Elastic Actuator spring. Units are Nm/rad", yoVariableRegistry);
    private final YoVariable motorSaturation = new YoVariable("motorSaturation" + suffix,
	    "True when the command motor torque is greater than peak motor torque" + " or the peak amlifier current", YoVariableType.BOOLEAN,
	    yoVariableRegistry);
    private final YoVariable kpSEA = new YoVariable("kpSEA" + suffix,
	    "SEA proportional gain from error in torque to torque applied to the motor. Units are Nm/Nm.", yoVariableRegistry);
    private final YoVariable kdSEA = new YoVariable("kdSEA" + suffix,
	    "SEA derivative gain from error in torque to torque applied to the motor. Units are Nm/(Nm/sec)", yoVariableRegistry);
    private final YoVariable gearboxDamping = new YoVariable("gearboxDamping" + suffix, "Viscous Friction in the gearbox due to velocity limit or grease",
	    yoVariableRegistry);
    private final YoVariable gearboxConstraintErrorAtOutputDot = new YoVariable("gearboxConstraintErrorAtOutputDot" + suffix, yoVariableRegistry);
    private final YoVariable gearboxConstraintErrorAtOutput = new YoVariable("gearboxConstraintErrorAtOutput" + suffix,
	    "The error between the motor shaft geared" + "down and the gearbox output, rad", yoVariableRegistry);
    
    private final YoVariable callCounterAC = new YoVariable("callCounterAC" + suffix, YoVariableType.INT, yoVariableRegistry);
    private final YoVariable callCounterTorqueControl = new YoVariable("callCounterTorqueControl" + suffix, YoVariableType.INT, yoVariableRegistry);

    private final SeriesElasticActuatorModel seriesElasticActuatorModel;

    private boolean simulateEncoders = false;
    private boolean enforceGearboxSpeedLimit = false;
    private boolean enforceMotorTorqueLimit = false;
    private boolean useGearboxEfficiency = false;
    
    private final AlphaFilteredYoVariable gearboxOutputVelocityFilt;
    private final AlphaFilteredYoVariable actuatorOutputVelocityFilt;
    private final AlphaFilteredYoVariable springDelfVelocityFilt;

    private final int simulationTicksPerControlTicks;
  
    private final HarmonicDriveEfficiencyCalculator harmonicDriveEfficiencyCalculator;
    private final SimulatedMotor simulatedMotor;
    
    public ActuatorController(SeriesElasticActuatorModel seriesElasticActuatorModel,  
	    double SEA_SpringConstant, double controlLoopPeriod, double simulationPeriod)
    {
	this.seriesElasticActuatorModel = seriesElasticActuatorModel;
	
	simulationTicksPerControlTicks = (int) Math.round(controlLoopPeriod/simulationPeriod);
	
	gearboxEfficiency.val = 1.0;
	
	tauDesired.val = 0.0; // 1.0;

	// wn.val = 40.0 * 2.0 * Math.PI;
	wn.val = 20.0 * 2.0 * Math.PI;
	zeta.val = 0.7; // 1.0;

	// NASA wrist spring is 323.0
	springConstant.val = SEA_SpringConstant; // Use a Units of Nm/rad

	// springConstant.val = 1000.0;
	springDamping.val = 1.0;
	gearboxDamping.val = 1000.0;

	if (USE_MOTOR)
	    setupWithMotor();
	else
	    setupWithoutMotor();

	// Display the theoretical bode diagram:
	double[] numerator = new double[]
	{ wn.val * wn.val };
	double[] denominator = new double[]
	{ 1.0, 2.0 * zeta.val * wn.val, wn.val * wn.val };
	double[] omega = SeriesElasticActuatorLinearDynamicSystem.generateLinearSpace(2000, 1.2, 0.2);
	TransferFunction linearTransferFunction = new TransferFunction(numerator, denominator);
	String temp = "2nd order Transfer Function. wn = " + wn.val / 6.28 + " Hz, zeta = " + zeta.val;
	
	boolean showBodePlot = false;
	if (showBodePlot)
	    BodePlotConstructor.plotBodeForTransferFunction(temp, linearTransferFunction, omega);

	double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequency(80.0, controlLoopPeriod);
	gearboxOutputVelocityFilt = new AlphaFilteredYoVariable("gearboxOutputVelocityFilt" + suffix, yoVariableRegistry, alpha);
	actuatorOutputVelocityFilt = new AlphaFilteredYoVariable("actuatorOutputVelocityFilt" + suffix, yoVariableRegistry, alpha);
	springDelfVelocityFilt = new AlphaFilteredYoVariable("springDelfVelocityFilt" + suffix, yoVariableRegistry, alpha);
	
	harmonicDriveEfficiencyCalculator = new HarmonicDriveEfficiencyCalculator(yoVariableRegistry);
	
	simulatedMotor = new SimulatedMotor(yoVariableRegistry, SEAParameters.PEAK_MOTOR_TORQUE, 
		SEAParameters.MAXIMUM_AMPLIFIER_CURRENT, SEAParameters.CONTINUOUS_AMPLIFIER_CURRENT, SEAParameters.TORQUE_SENSITIVITY, controlLoopPeriod);
	
	instanceCounter++;
    }
    
    public void setUseGearboxEfficiency(boolean useGearboxEfficiency)
    {
       this.useGearboxEfficiency = useGearboxEfficiency;  
    }

    public void setSimulateEncoders(boolean simulateEncoders)
    {
       this.simulateEncoders = simulateEncoders;  
    }

    public void setEnforceGearboxSpeedLimit(boolean enforceGearboxSpeedLimit)
    {
	if (enforceGearboxSpeedLimit)
	{
	    System.err.println("The speed limit is captured in the useGearboxEfficiency, are you sure you want to enforceGearboxSpeedLimit?");
	}
	
	this.enforceGearboxSpeedLimit = enforceGearboxSpeedLimit;  
    }

    public void setEnforceMotorTorqueLimit(boolean enforceMotorTorqueLimit)
    {
       this.enforceMotorTorqueLimit = enforceMotorTorqueLimit;  
    }

    private void setupWithMotor()
    {
	setTauGainsWithMotor(wn.val, zeta.val);

	kpGear.val = SEAParameters.kpGearDefault;
	kdGear.val = SEAParameters.kdGearDefault;
    }

    private void setupWithoutMotor()
    {
	setTauGainsWithoutMotor(wn.val, zeta.val);

	kpGear.val = 0.0;
	kdGear.val = 0.0;
    }

    private void setTauGainsWithMotor(double naturalFreq, double dampingRatio)
    {
	double IequivWithMotor = SEAParameters.GEARBOX_OUTPUT_I + SEAParameters.MOTOR_ROTOR_I * SEAParameters.GEAR_RATIO * SEAParameters.GEAR_RATIO;

	setTauGains(naturalFreq, dampingRatio, IequivWithMotor);
    }

    private void setTauGainsWithoutMotor(double naturalFreq, double dampingRatio)
    {
	double IequivNoMotor = SEAParameters.GEARBOX_OUTPUT_I;
	setTauGains(naturalFreq, dampingRatio, IequivNoMotor);
    }

    private void setTauGains(double naturalFreq, double dampingRatio, double Iequiv)
    {
	kpSEA.val = (naturalFreq * naturalFreq * Iequiv - springConstant.val) / (springConstant.val);
	kdSEA.val = 2.0 * dampingRatio * naturalFreq * Iequiv / (springConstant.val);
    }

    public void setTorqueFeedbackGains(double kp, double kd)
    {
	kpSEA.val = kp;
	kdSEA.val = kd;
    }
    
    public void setTauGains(double naturalFreq, double dampingRatio)
    {
	if (USE_MOTOR)
	{
	    setTauGainsWithMotor(naturalFreq, dampingRatio);
	}
	else
	{
	    setTauGainsWithoutMotor(naturalFreq, dampingRatio);
	}
    }

    public void doControl()
    {
	if (callCounterAC.getIntegerValue() % simulationTicksPerControlTicks == 0)
	{
	    callCounterTorqueControl.val++;
	    seriesElasticActuatorModel.updateEncoderValues();

	    updateFilteredVelocityValues();

	    doSpringAndLoadTorque();

	    // This is the torque control
	    computeTorqueError();
	    tauDrive.val = tauDesired.val + kpSEA.val * tauError.val + kdSEA.val * tauErrorDot.val;
	    
	    tauDrive.val = simulatedMotor.getActualTorqueToMotor(tauDrive.val/ SEAParameters.GEAR_RATIO, enforceMotorTorqueLimit) * SEAParameters.GEAR_RATIO;
	    
	    if (useGearboxEfficiency)
	    {
	       gearboxEfficiency.val = computeGearboxEfficiency();
	    }
	    else
	    {
	       gearboxEfficiency.val = 1.0;
	    }
	    
	    tauDriveAfterLosses.val = gearboxEfficiency.val * tauDrive.val;
	}

	if (USE_MOTOR)
	    doControlWithMotorInModel();
	else
	    doControlWithoutMotorInModel();
	
	callCounterAC.val++;
    }
    
    public double computeGearboxEfficiency()
    {
       return harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(
		    seriesElasticActuatorModel.getGearboxInputShaftVelocity(),
		    tauDrive.val);
//		    seriesElasticActuatorModel.getTorqueOnGearboxMass());
    }
    
  
//    private double clipTorqueIfAboveLimits(double gearboxOutputTorque)
//    {
//
//	// Note that the torque passed in is after the gear ratio
//	if (gearboxOutputTorque > SEAParameters.PEAK_DRIVE_TORQUE)
//	{
//	    motorSaturation.set(true);
//
//	    return SEAParameters.PEAK_DRIVE_TORQUE;
//	} else if (gearboxOutputTorque < -SEAParameters.PEAK_DRIVE_TORQUE)
//	{
//	    motorSaturation.set(true);
//
//	    return -SEAParameters.PEAK_DRIVE_TORQUE;
//	} else
//	{
//	    motorSaturation.set(false);
//
//	    return gearboxOutputTorque;
//	}
//    }

    private void doControlWithMotorInModel()
    {
	doGearbox();

	double totalMotorTorque = (tauDriveAfterLosses.val - tauGearboxConstraint.val) / SEAParameters.GEAR_RATIO;
	seriesElasticActuatorModel.setTorqueOnMotorMass(totalMotorTorque);
    }

    private void doControlWithoutMotorInModel()
    {
	double totalMotorTorque = 0.0; // Just ignore the motor mass.
	seriesElasticActuatorModel.setTorqueOnMotorMass(totalMotorTorque);

	tauGearboxDamping.val = getGearboxDampingTorque(seriesElasticActuatorModel.getGearboxOutputShaftVelocity());

	totalGearboxOutputTorque.val = tauDriveAfterLosses.val - tauGearboxDamping.val;
	seriesElasticActuatorModel.setTorqueOnGearboxMass(totalGearboxOutputTorque.val);
    }

    private void doGearbox()
    {
	gearboxConstraintErrorAtOutput.val = seriesElasticActuatorModel.getMotorShaftPosition() / SEAParameters.GEAR_RATIO
		- seriesElasticActuatorModel.getGearboxOutputShaftPosition();

	gearboxConstraintErrorAtOutputDot.val = seriesElasticActuatorModel.getMotorShaftVelocity() / SEAParameters.GEAR_RATIO
		- seriesElasticActuatorModel.getGearboxOutputShaftVelocity();

	tauGearboxConstraint.val = kpGear.val * gearboxConstraintErrorAtOutput.val + kdGear.val * gearboxConstraintErrorAtOutputDot.val;

	tauGearboxDamping.val = getGearboxDampingTorque(seriesElasticActuatorModel.getGearboxOutputShaftVelocity());

	totalGearboxOutputTorque.val = tauGearboxConstraint.val - tauGearboxDamping.val;
	seriesElasticActuatorModel.setTorqueOnGearboxMass(totalGearboxOutputTorque.val);
    }

    private void doSpringAndLoadTorque()
    {
	springDeflection.val = seriesElasticActuatorModel.getSpringDeflection();
	springDeflectionDot.val = seriesElasticActuatorModel.getSpringDeflectionRate();
	
	springDeflectionEncoder.val = seriesElasticActuatorModel.getSpringDeflectionFromEncoder();
	springDeflectionEncoderDot.val = seriesElasticActuatorModel.getSpringDeflectionRateFromEncoder();
	springDelfVelocityFilt.update(springDeflectionEncoderDot.val);
	
	
	if (simulateEncoders)
	{
	    tauActual.val = -springConstant.val * springDeflectionEncoder.val;
//	    tauActualDot.val = -springConstant.val * springDeflectionEncoderDot.val;
	    tauActualDot.val = -springConstant.val * springDelfVelocityFilt.val;
	} 
	else
	{
	    tauActual.val = -springConstant.val * springDeflection.val;
	    tauActualDot.val = -springConstant.val * springDeflectionDot.val;
	}

	double springDampingTorque = springDeflectionDot.val * springDamping.val;

	double totalLoadTorque = tauActual.val - springDampingTorque;
	seriesElasticActuatorModel.setTorqueOnLoadMass(totalLoadTorque);
    }

    private void computeTorqueError()
    {
	tauError.val = tauDesired.val - tauActual.val;
	tauErrorDot.val = -tauActualDot.val;

	// tauErrorDot.val = tauDesiredDot.val - tauActualDot.val;
    }
    
    private void updateFilteredVelocityValues()
    {
	gearboxOutputVelocityFilt.update(seriesElasticActuatorModel.getGearboxOutputShaftVelocityFromEncoder());
	actuatorOutputVelocityFilt.update(seriesElasticActuatorModel.getActuatorOutputVelocityFromEncoder());
    }

    public void setDesiredTorque(double desiredTorque)
    {
	tauDesired.val = desiredTorque;
    }

    // public void setDesiredTorqueDot(double desiredTorqueDot)
    // {
    // tauDesiredDot.val = desiredTorqueDot;
    // }

    private double getGearboxDampingTorque(double gearboxOutputVelocity)
    {
	if (!enforceGearboxSpeedLimit)
	    return 0.0;

	// This should return something of the same sign as the velocity

	double overSpeed = Math.abs(gearboxOutputVelocity) - SEAParameters.MAXIMUM_GEARBOX_OUTPUT_SPEED;
	if (overSpeed < 0.0)
	    return 0.0;
	else
	{
	    double ret = overSpeed * gearboxDamping.val * Math.signum(gearboxOutputVelocity);

	    // System.out.println("ret=" + ret);

	    return ret;
	}
    }

    public YoVariableRegistry getYoVariableRegistry()
    {
	return yoVariableRegistry;
    }
}
