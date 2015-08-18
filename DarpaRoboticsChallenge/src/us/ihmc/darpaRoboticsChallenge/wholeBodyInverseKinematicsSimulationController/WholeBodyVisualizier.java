package us.ihmc.darpaRoboticsChallenge.wholeBodyInverseKinematicsSimulationController;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class WholeBodyVisualizier
{
   private final SimulationConstructionSet scs;
   public WholeBodyVisualizier(DRCRobotModel robotModel)
   {
      //model = AtlasRobotModelFactory.selectSimulationModelFromFlag(args);
   
      SDFRobot simulatedRobot = robotModel.createSdfRobot(false);

      scs = new SimulationConstructionSet(simulatedRobot);
      scs.startOnAThread();
   }
}
