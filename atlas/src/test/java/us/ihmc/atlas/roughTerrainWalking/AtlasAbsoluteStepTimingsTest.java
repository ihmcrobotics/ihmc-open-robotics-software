package us.ihmc.atlas.roughTerrainWalking;

import java.io.InputStream;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarAbsoluteStepTimingsTest;
import us.ihmc.robotics.Assert;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class AtlasAbsoluteStepTimingsTest extends AvatarAbsoluteStepTimingsTest
{
   @Override
   @Test
   public void testTakingStepsWithAbsoluteTimings()
   {
      super.testTakingStepsWithAbsoluteTimings();
   }

   @Override
   @Test
   public void testMinimumTransferTimeIsRespected()
   {
      super.testMinimumTransferTimeIsRespected();
   }

   @Override
   @Test
   public void testPausingWalkDuringLongTransfers()
   {
      super.testPausingWalkDuringLongTransfers();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream overwrites = AvatarAbsoluteStepTimingsTest.class.getResourceAsStream("/absolute_step_timing_test.xml");
            Assert.assertNotNull(overwrites);
            return overwrites;
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }
}
