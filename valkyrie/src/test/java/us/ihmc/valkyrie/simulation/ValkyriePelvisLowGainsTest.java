package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

@Disabled
@Tag("fast")
public class ValkyriePelvisLowGainsTest extends DRCPelvisLowGainsTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS)
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
   @Test
   public void testStandingWithLowPelvisOrientationGains()
   {
      super.testStandingWithLowPelvisOrientationGains();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(YoVariableHolder yoVariableHolder)
   {
      return (YoDouble) yoVariableHolder.findVariable("HighLevelHumanoidControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.v1PelvisAxisAngleOrientationController",
                                                      "v1PelvisOrientationErrorMagnitude");
   }
}
