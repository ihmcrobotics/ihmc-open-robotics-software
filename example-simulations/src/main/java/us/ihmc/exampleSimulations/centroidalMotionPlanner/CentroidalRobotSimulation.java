package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import us.ihmc.commons.thread.ThreadTools;
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
      CentroidalRobotSimulation sim = new CentroidalRobotSimulation();
      sim.runSimulation();
   }

   public void runSimulation()
   {
      CentroidalDynamicsRobot robot = new CentroidalDynamicsRobot("TestRobot");
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(createGUI, bufferSize);
      Robot scsRobot = robot.getSCSRobot();
      SimulationConstructionSet scs = new SimulationConstructionSet(scsRobot, parameters);
      scs.setPlaybackRealTimeRate(1.0);
      scs.startOnAThread();
      BlockingSimulationRunner runner = new BlockingSimulationRunner(scs, 4.0);
      try
      {
         runner.simulateAndBlockAndCatchExceptions(4.0);
         ThreadTools.sleepForever();
      }
      catch (SimulationExceededMaximumTimeException e)
      {
         e.printStackTrace();
      }
   }
}