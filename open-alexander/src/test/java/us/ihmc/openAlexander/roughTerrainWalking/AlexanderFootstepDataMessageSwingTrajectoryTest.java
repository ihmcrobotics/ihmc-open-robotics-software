package us.ihmc.openAlexander.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalPropertiesV0;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarFootstepDataMessageSwingTrajectoryTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class AlexanderFootstepDataMessageSwingTrajectoryTest extends AvatarFootstepDataMessageSwingTrajectoryTest
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
      return new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, target);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Override
   public double getLegLength()
   {
      AlexanderPhysicalProperties physicalProperties = new AlexanderPhysicalPropertiesV0();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }
}
