package us.ihmc.atlas.roughTerrainWalking;

import java.io.InputStream;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarAbsoluteStepTimingsTest;
import us.ihmc.avatar.roughTerrainWalking.AvatarFootstepDataMessageSwingTrajectoryTest;
import us.ihmc.robotics.Assert;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

@Tag("humanoid-rough-terrain")
public class AtlasFootstepDataMessageSwingTrajectoryTest extends AvatarFootstepDataMessageSwingTrajectoryTest
{
   @Override
   @Test
   public void testSwingTrajectoryInWorld() 
   {
      super.testSwingTrajectoryInWorld();
   }

   @Override
   @Test
   public void testSwingTrajectoryTouchdownSpeed() 
   {
      setPushAndAdjust(false);
      super.testSwingTrajectoryTouchdownSpeed();
   }

   @Override
   @Test
   public void testSwingTrajectoryTouchdownWithAdjustment() 
   {
      setPushAndAdjust(true);
      super.testSwingTrajectoryTouchdownWithAdjustment();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      RobotTarget target = RobotTarget.SCS;
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false)
      {
         @Override
         public InputStream getParameterOverwrites()
         {
            InputStream overwrites = AvatarAbsoluteStepTimingsTest.class.getResourceAsStream("/zero_touchdown_acceleration.xml");
            Assert.assertNotNull(overwrites);
            return overwrites;
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   public double getLegLength()
   {
      AtlasPhysicalProperties physicalProperties = new AtlasPhysicalProperties();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }
}
