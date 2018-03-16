package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class CentroidalRobotSimulation
{
   public static final boolean createGUI = true;
   public static final int bufferSize = 1 << 20;
   
   public static void main(String args[])
   {
      System.out.println(bufferSize);
      CentroidalRobotSimulation sim = new CentroidalRobotSimulation();
      sim.runSimulation();
   }
   
   public void runSimulation()
   {
      CentroidalDynamicsRobot robot = new CentroidalDynamicsRobot("TestRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(createGUI, bufferSize);
      Robot scsRobot = robot.getSCSRobot();
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobot, parameters);
      scs.setPlaybackRealTimeRate(0.025);
      scs.startOnAThread();
      BlockingSimulationRunner runner = new BlockingSimulationRunner(scs, 100.0);
      try
      {
         runner.simulateAndBlockAndCatchExceptions(10.0);
      }
      catch (SimulationExceededMaximumTimeException e)
      {
         e.printStackTrace();
      }
   }
}
