package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AtlasSDFViewer
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotModel.createSdfRobot(false));
      scs.startOnAThread();
   }
}
