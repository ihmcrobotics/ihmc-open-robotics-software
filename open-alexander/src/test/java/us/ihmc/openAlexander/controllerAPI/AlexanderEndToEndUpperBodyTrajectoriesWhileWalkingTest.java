package us.ihmc.openAlexander.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndUpperBodyTrajectoriesWhileWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndUpperBodyTrajectoriesWhileWalkingTest extends EndToEndUpperBodyTrajectoriesWhileWalkingTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   protected double getArmJointRangeOfMotionLimit()
   {
      return 0.5;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithRandomArmTrajectoryMovements() throws Exception
   {
      super.testWalkingWithRandomArmTrajectoryMovements();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithArmsHoldingInFeetFrame() throws Exception
   {
      super.testWalkingWithArmsHoldingInFeetFrame();
   }
}
