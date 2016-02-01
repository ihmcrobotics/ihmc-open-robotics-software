package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 
 2 DOF Robot (2 legs)
    
    --The robot   
          a) has a support leg and a leg that can swing
          b) is stable on just one foot
    
    -- We can control:
          1) Body height: applying a torque at the knee of the support leg
          2) Leg swing: applying a torque at the hip of the lifted leg
          3) Leg flexion and extension: applying a force at the knee of the lifted leg
          4) Body pitch: applying a torque at the hip of the support leg
          
   -- The ground contact points have sphere shapes associated to them for visualization
 
 **/

public class Step3Simulation {

	/** 
	 * Variables
	 */
	SimulationConstructionSet sim;
	private double deltaT = 0.00001;
	private int recordFrequency = 10;
	
	/**
	 * Constructor
	 */
	public Step3Simulation() {
		
		// Robot
		Step3Robot v3Robot = new Step3Robot();

		// Controller
		Step3Controller v3Controller = new Step3Controller(v3Robot, "v3Robot", deltaT);
		v3Robot.setController(v3Controller);

		// Simulation Object
		sim = new SimulationConstructionSet(v3Robot);
		sim.setGroundVisible(true);
		sim.setDT(deltaT, recordFrequency);
		sim.setCameraFix(0.0, 0.025, 1.05);
		sim.setCameraPosition(12.6, -15.4, 2.3);
		sim.changeBufferSize(32000);
		sim.selectConfiguration("Step3GUI.guiConf");
		
		Thread myThread = new Thread(sim);
		myThread.start();
	}
	
	/**
	 * Main 
	 */
	public static void main(String[] args) {
		new Step3Simulation();
	}

}
