package us.ihmc.atlas.roughTerrainWalking;

import java.io.InputStream;

import org.junit.Assert;
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
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream overwrites = AvatarFootWobbleTest.class.getResourceAsStream("/foot_wobble_test.xml");
            Assert.assertNotNull(overwrites);
            return overwrites;
         }
      };
   }

   @Override
   @Test(timeout = 300000)
   public void testICPReplanningInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testICPReplanningInSwing();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

}
