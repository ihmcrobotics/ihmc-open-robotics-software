package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.CameraConfiguration;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.graphics.*;

import java.util.ArrayList;

import us.ihmc.moonwalking.models.SeriesElasticActuator.DesiredTorqueController.TestToConduct;


public class SeriesElasticActuatorSimulation
{
    public static final double DT = 0.000001;

    public static final double PEAK_TORQUE_TO_TRACK = 200.0; //[Nm]
    public static final double PEAK_POSITION_TO_TRACK = 0.2; //1.57; //[rad]
    
    public SeriesElasticActuatorSimulation()
    {
	startSim();
    }

    private void startSim()
    {
	
	DesiredTorqueController.testToConduct = TestToConduct.TORQUE_CHIRP;
//	DesiredTorqueController.testToConduct = TestToConduct.POSITION_SQUARE;
//	DesiredTorqueController.testToConduct = TestToConduct.POSITION_CHIRP;
	
	
	if (DesiredTorqueController.testToConduct == TestToConduct.TORQUE_CHIRP ||
	    DesiredTorqueController.testToConduct == TestToConduct.TORQUE_SQUARE)
	{
	    //clamp the output
	    SEAParameters.LOAD_MASS = 1e9;
	    SEAParameters.recalculateLoadInteria();
	    System.out.println("Clmaping output for torque test");
	    System.out.println("SEAParameters.LOAD_INERTIA=" + SEAParameters.LOAD_INERTIA);
	}
	
	SEAParameters sEAParameters = new SEAParameters();
	
	


	SeriesElasticActuatorModel seriesElasticActuatorModelZero, seriesElasticActuatorModelOne;
	
	// Zero Actuator
	double forceControlDT_Zero = 0.0001;
	{
	    double desiredTorqueUpdateDT = 0.002;
	    int simulationTciksPerDesiredTorqueUpdateTick = (int) (Math.round(desiredTorqueUpdateDT / DT));
	    
	    seriesElasticActuatorModelZero = new SeriesElasticActuatorModel(SEAParameters.APS_ENCODER_COUNTS_PER_RADIAN);
	    ActuatorController actuatorControllerZero = new ActuatorController(seriesElasticActuatorModelZero, 
		    SEAParameters.springConstantDefault * 10.0,
		    forceControlDT_Zero, DT);

	    double naturalFreq = 30.0 * 2.0 * Math.PI;
	    double dampingRatio = 0.7; 
	    actuatorControllerZero.setTauGains(naturalFreq, dampingRatio);
	    
	    actuatorControllerZero.setSimulateEncoders(true);
	    actuatorControllerZero.setEnforceMotorTorqueLimit(true);
	    actuatorControllerZero.setUseGearboxEfficiency(true);

	    //The speed limit is already taken into account with the gearbox efficiency
	    actuatorControllerZero.setEnforceGearboxSpeedLimit(false);

	    
	    //this is to turn off torque control
//	    actuatorControllerZero.setTorqueFeedbackGains(0.0, 0.0);
	    
	    seriesElasticActuatorModelZero.setController(actuatorControllerZero, 1); //simulationTicksPerControlTick_Zero);

	    DesiredTorqueController desiredTorqueControllerZero = new DesiredTorqueController(actuatorControllerZero, seriesElasticActuatorModelZero);
	    seriesElasticActuatorModelZero.setController(desiredTorqueControllerZero, simulationTciksPerDesiredTorqueUpdateTick);
	}

	// One Actuator
	double forceControlDT_One = 0.0001;
	{ 
	    double desiredTorqueUpdateDT = 0.002;
	    int simulationTciksPerDesiredTorqueUpdateTick = (int) (Math.round(desiredTorqueUpdateDT / DT));
	    
	    seriesElasticActuatorModelOne = new SeriesElasticActuatorModel(SEAParameters.APS_ENCODER_COUNTS_PER_RADIAN);
	    ActuatorController actuatorControllerOne = new ActuatorController(seriesElasticActuatorModelOne, 
		    SEAParameters.springConstantDefault,
		    forceControlDT_One, DT);
	    
	    double naturalFreq = 20.0 * 2.0 * Math.PI;
	    double dampingRatio = 0.7; 
	    actuatorControllerOne.setTauGains(naturalFreq, dampingRatio);

	   
	    actuatorControllerOne.setSimulateEncoders(true);
	    actuatorControllerOne.setEnforceMotorTorqueLimit(true);
	    actuatorControllerOne.setUseGearboxEfficiency(true);

	    //The speed limit is already taken into account with the gearbox efficiency
	    actuatorControllerOne.setEnforceGearboxSpeedLimit(false);

	    
	    //this is to turn off torque control	
//	    actuatorControllerOne.setTorqueFeedbackGains(0.0, 0.0);

	    
	    seriesElasticActuatorModelOne.setController(actuatorControllerOne, 1) ;//simulationTicksPerControlTick_One);
	    
	    DesiredTorqueController desiredTorqueControllerOne = new DesiredTorqueController(actuatorControllerOne, seriesElasticActuatorModelOne);
	    seriesElasticActuatorModelOne.setController(desiredTorqueControllerOne, simulationTciksPerDesiredTorqueUpdateTick);
	}

	Robot[] robots = new Robot[]
	{ seriesElasticActuatorModelZero, seriesElasticActuatorModelOne};
	// Robot[] robots = new Robot[] {seriesElasticActuatorModelZero};

	SimulationConstructionSet scs = new SimulationConstructionSet(robots);
	scs.changeBufferSize(32000);
	
	// scs.
	setUpGUI(scs);
	setUpGUIForAmplifierStatus(scs);
	// scs.selectConfiguration("Control_0");
	scs.selectConfiguration("Control_Comp");

	// scs.selectConfiguration("InitalStudy");

	ArrayList<DynamicGraphicObjectsList> list = seriesElasticActuatorModelOne.getDynamicGraphicObjectsList();

	DynamicGraphicObjectsListRegistry dynamicGraphicObjectsRegistry = new DynamicGraphicObjectsListRegistry();
	dynamicGraphicObjectsRegistry.registerDynamicGraphicObjectsLists(list);

	dynamicGraphicObjectsRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

	// scs.setGroundVisible(false);

	int recordFrequency_Zero = (int) Math.round(forceControlDT_Zero / DT);
	int recordFrequency_One = (int) Math.round(forceControlDT_Zero / DT);
	int recordFrequency = Math.min(recordFrequency_Zero, recordFrequency_One) * 10;
	scs.setDT(DT, recordFrequency);
	// scs.setDT(DT, 2000);

	Thread thread = new Thread(scs);
	thread.setName("SimulationConstructionSet");
	thread.start();
    }

    private void setUpGUI(SimulationConstructionSet scs)
    {
	CameraConfiguration view1 = new CameraConfiguration("view1");
	view1.setCameraDolly(false, true, true, false);
	view1.setCameraTracking(false, true, true, false);
	view1.setCameraPosition(2.35, -1.77, 0.8);
	view1.setCameraFix(0.03, 0.008, 0.18);
	scs.setupCamera(view1);

	scs.selectCamera("view1");

	String zero = "_" + 0;
	String one = "_" + 1;

	scs.setupGraphGroup("Control" + zero, new String[][][]
	{
	{
	{ "t" },
	{ "auto" } },
	{
	{ "tauDesired" + zero, "tauActual" + zero, "tauDrive" + zero },
	{ "auto" } },
	{
	{ "outputPosDesired" + zero, "outputPosActual" + zero },
	{ "auto" } },
	{
	{ "qd_gearboxOutputShaft" + zero },
	{ "auto" } },
	{
	{ "motorSaturation" + zero },
	{ "" } }, }, 1);

	scs.setupGraphGroup("Control_Comp", new String[][][]
	{
	{
	{ "t" },
	{ "auto" } },
	{
	{ "t" },
	{ "auto" } },
	{
	{ "tauDesired" + zero, "tauActual" + zero, "tauDrive" + zero },
	{ "auto" } },
	{
	{ "tauDesired" + one, "tauActual" + one, "tauDrive" + one },
	{ "auto" } },
	{
	{ "outputPosDesired" + zero, "outputPosActual" + zero },
	{ "auto" } },
	{
	{ "outputPosDesired" + one, "outputPosActual" + one },
	{ "auto" } },
	{
	{ "qd_gearboxOutputShaft" + zero, "qd_gearboxOutputShaft" + one },
	{ "auto" } },
	{
	{ "qd_gearboxOutputShaft" + one },
	{ "auto" } },
	{
	{ "motorLimit" + zero },
	{ "" } },
	{
	{ "motorLimit" + one },
	{ "" } },
	{
	{ "tauDrive" + zero, "tauDrive" + one },
	{ "auto" } }, 
	{
	{ "gearboxEfficiency" + zero, "gearboxEfficiency" + one },
	{ "auto" } }, 
	
	}, 2);

	scs.setupGraphGroup("Encoders", new String[][][]
	{
	{
	{ "gearboxOutputPositionFromEncoder" + zero, "q_gearboxOutputShaft" + zero },
	{ "auto" } },
	{
	{ "gearboxOutputVelocityFromEncoder" + zero, "qd_gearboxOutputShaft" + zero, "gearboxOutputVelocityFilt" + zero },
	{ "auto" } },
	{
	{ "springDeflectionEncoder" + zero, "springDeflection" + zero },
	{ "auto" } },
	{
	{ "springDeflectionEncoderDot" + zero, "springDeflectionDot" + zero, "springDelfVelocityFilt" + zero },
	{ "auto" } },

	}, 1);

	//
	// GraphConfiguration phasePlot = new GraphConfiguration("phasePlot",
	// GraphConfiguration.AUTO_SCALING);
	// phasePlot.setPlotType(GraphConfiguration.PHASE_PLOT);
	// scs.setupGraphConfigurations(new GraphConfiguration[]{phasePlot});
	//
	// scs.setupGraphGroup("PhasePortrait", new String[][][]
	// {
	// {{"q_pitch", "qd_pitch"},{"phasePlot"}},
	// }, 1);
	//
	// scs.setupGraphGroup("InitalStudy", new String[][][]
	// {
	// {{"error"},{"auto"}},
	// {{"q_pitch"},{"auto"}},
	// {{"qd_pitch"},{"auto"}},
	// {{"q_ankle"},{"auto"}},
	// {{"qd_ankle"},{"auto"}},
	// } , 1);
	//
	// scs.setupConfiguration("InitalStudy", "all", "InitalStudy",
	// "Control");
	//
	// scs.setupVarGroup("PhasePortrait", new String[]
	// {"t", "q_pitch", "qd_pitch", "settled"});
	//
	scs.setupEntryBoxGroup("Control" + zero, new String[]
	{ "desiredTorqueForController" + zero, "springDamping" + zero, "generatorTau" + zero + "Offset", "generatorTau + zero +  Amp",
		"generatorTau" + zero + "Freq", "generatorPos" + zero + "Offset", "generatorPos" + zero + "Amp", "generatorPos" + zero + "Freq",
		"kpPosition" + zero, "kdPosition" + zero, "kpPosition" + one, "kdPosition" + one,
		"kpSEA" + zero, "kdSEA" + zero, "kpSEA" + one, "kdSEA" + one,});

	//
	scs.setupConfiguration("Control" + zero, "all", "Control" + zero, "Control" + zero);
	scs.setupConfiguration("Control_Comp", "all", "Control_Comp", "Control" + zero);

	//
	// scs.setupConfiguration("Control", "all", "Control", "Control");
    }
    
    private void setUpGUIForAmplifierStatus(SimulationConstructionSet scs)
    {
	String zero = "_" + 0;
	String one = "_" + 1;
	
	scs.setupGraphGroup("Amplifiers", new String[][][]
	                                                 {
		{
	{ "motorLimit" + zero },
	{ "auto" } },
	{
	{ "motorLimit" + one},
	{ "auto" } },
	{
	{ "amplifierTemperature" + zero, "maxAmplifierTemperature" + zero, },
	{ "auto" } },
	{
	{ "amplifierTemperature" + one, "maxAmplifierTemperature" + one,},
	{ "auto" } },
	{
	{ "amplifierTemperatureStatus" + zero,},
	{ "auto" } },
	{
	{ "amplifierTemperatureStatus" + one},
	{ "auto" } },
	{
	{ "availableCurrent" + zero, "motorCurrent" + zero},
	{ "auto" } },
	{
	{ "availableCurrent" + one, "motorCurrent" + one},
	{ "auto" } },
	
	}, 2);
    }
    

    public static void main(String[] args)
    {
	SeriesElasticActuatorSimulation seriesElasticActuatorSimulation = new SeriesElasticActuatorSimulation();
    }
}
