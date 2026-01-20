package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.zulu.parameters.model.ZuluPhysicalPropertiesV0;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarFootstepDataMessageSwingTrajectoryTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class ZuluFootstepDataMessageSwingTrajectoryTest extends AvatarFootstepDataMessageSwingTrajectoryTest
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
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, target);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Override
   public double getLegLength()
   {
      ZuluPhysicalProperties physicalProperties = new ZuluPhysicalPropertiesV0();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }
}
