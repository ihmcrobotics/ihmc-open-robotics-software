package us.ihmc.exampleSimulations.yoFilteredDouble.emptyRobotSCS;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFilteredDoubleSimulation {
	
	private YoRegistry registry;
	private double dt_ns = 1000000.0; // 100 Hz 
	
	public YoFilteredDoubleSimulation() {
		
		// 1.0 Create YoRegistry to store YoFiltered Doubles in for the simulation.
		registry = new YoRegistry("yoFilteredDoubleTest");
		
		Robot robot = new Robot("EmptyRobot");
		SimulationConstructionSet scs = new SimulationConstructionSet();
		YoFilteredDoubleController controller = new YoFilteredDoubleController(registry, dt_ns);
		robot.setController(controller);
		scs.setRobot(robot);
		scs.setDT(dt_ns/1000000000.0, 1);
		scs.changeBufferSize(16342*10);
		scs.hideViewport(); 
		scs.setFastSimulate(true, 10);
		scs.setSimulateNoFasterThanRealTime(true); 
		scs.startOnAThread();
	}
	
	public static void main(String[] args)
	{
		YoFilteredDoubleSimulation sim = new YoFilteredDoubleSimulation();
	}
}

