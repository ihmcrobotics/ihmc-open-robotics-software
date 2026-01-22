package us.ihmc.zulu.networkProcessor.footstepPostProcessing;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPostProcessing.AvatarPostProcessingTests;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class ZuluPostProcessingTest extends AvatarPostProcessingTests
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS)
      {
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ZuluWalkingControllerParameters(getRobotVersion(), RobotTarget.SCS, getJointMap(), getPhysicalProperties(), getContactPointParameters())
            {
               @Override
               public double maximumHeightAboveAnkle()
               {
                  return 0.89;
               }
            };
         }

         @Override
         public SwingPlannerParametersBasics getSwingPlannerParameters()
         {
            SwingPlannerParametersBasics parametersBasics = new DefaultSwingPlannerParameters();
            parametersBasics.setDoInitialFastApproximation(true);
            parametersBasics.setMinimumSwingFootClearance(0.0);
            parametersBasics.setNumberOfChecksPerSwing(100);
            parametersBasics.setMaximumNumberOfAdjustmentAttempts(50);
            parametersBasics.setMaximumWaypointAdjustmentDistance(0.2);
            parametersBasics.setMinimumAdjustmentIncrementDistance(0.03);
            parametersBasics.setMinimumHeightAboveFloorForCollision(0.03);

            return parametersBasics;
         }
      };
   }

   @Override
   public String getLeftAnkleXName()
   {
      return "LEFT_ANKLE_X";
   }

   @Override
   public String getLeftAnkleYName()
   {
      return "LEFT_ANKLE_Y";
   }

   @Override
   public String getRightAnkleXName()
   {
      return "RIGHT_ANKLE_X";
   }

   @Override
   public String getRightAnkleYName()
   {
      return "RIGHT_ANKLE_Y";
   }

   protected static final double[] rightHandStraightSideJointAngles = new double[] {0.0, Math.toRadians(-75), 0.0, 0.0, 0.0, 0.0, 0.0};
   protected static final double[] leftHandStraightSideJointAngles = new double[] {0.0, Math.toRadians(75),  0.0, 0.0, 0.0, 0.0, 0.0};
   protected static final SideDependentList<double[]> straightArmConfigs = new SideDependentList<>();
   static
   {
      straightArmConfigs.put(RobotSide.LEFT, leftHandStraightSideJointAngles);
      straightArmConfigs.put(RobotSide.RIGHT, rightHandStraightSideJointAngles);
   }


   @Override
   public double[] getStraightArmConfig(RobotSide robotSide)
   {
      return straightArmConfigs.get(robotSide);
   }


   @Tag("humanoid-obstacle-2")
   @Override
   @Test
   public void testWalkingOffOfMediumPlatform()
   {
      super.testWalkingOffOfMediumPlatform();
   }

   @Disabled
   @Tag("humanoid-obstacle-2")
   @Override
   @Test
   public void testSwingOverPlanarRegions()
   {
      super.testSwingOverPlanarRegions();
   }

   @Tag("humanoid-obstacle-2")
   @Override
   @Test
   public void testWalkingOnStraightForwardLines()
   {
      super.testWalkingOnStraightForwardLines();
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }
}
