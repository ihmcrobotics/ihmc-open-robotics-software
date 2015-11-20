package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class FourBarLinkageSimulation {

	private Vector3d offsetWorld = new Vector3d(0.0, 0.0, 0.0);
	private static final double simDT = 1.0e-4;
	private static final int recordFrequency = 10;

	public FourBarLinkageSimulation()
	{
	// Parameters
	FourBarLinkageParameters robotParameters = new FourBarLinkageParameters();

	// Robot
	FourBarLinkageRobot robot = new FourBarLinkageRobot("basicFourBar", robotParameters, offsetWorld);

	// Controller

	// SCS
	SimulationConstructionSet scs = new SimulationConstructionSet(robot);
	scs.setDT(simDT, recordFrequency);
	scs.startOnAThread();
	}
	
	public static void main(String[] args) 
	{
		new FourBarLinkageSimulation();
	}
}
