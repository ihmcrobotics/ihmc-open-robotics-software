package us.ihmc.atlas.roughTerrainWalking;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarFootWobbleTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasFootWobbleTest extends AvatarFootWobbleTest
{
   private DRCRobotModel robotModel;

   @Before
   public void createRobotModel()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   @Test(timeout = 300000)
   public void testICPReplanning() throws SimulationExceededMaximumTimeException
   {
      super.testICPReplanning();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

}
