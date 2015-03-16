package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AtlasSDFViewer
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS_UNPLUGGED_V4, AtlasRobotModel.AtlasTarget.SIM, false);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotModel.createSdfRobot(false));
      scs.startOnAThread();
   }
}
