package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlexanderPelvisLowGainsTest extends DRCPelvisLowGainsTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS)
   {
      // Disable joint damping to make sure that damping isn't causing the problem.
      private static final boolean enableJointDamping = false;

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         return createHumanoidFloatingRootJointRobot(createCollisionMeshes, enableJointDamping);
      }
   };

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testStandingWithLowPelvisOrientationGains()
   {
      super.testStandingWithLowPelvisOrientationGains();
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(YoVariableHolder yoVariableHolder)
   {

      return (YoDouble) yoVariableHolder.findVariable(FeedbackControllerToolbox.class.getSimpleName(), "PELVIS_LINKErrorRotationVectorZ");
   }
}
