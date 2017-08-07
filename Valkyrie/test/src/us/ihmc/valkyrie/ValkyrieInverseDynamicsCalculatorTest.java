package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.DRCInverseDynamicsCalculatorTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.EXCLUDE})
public class ValkyrieInverseDynamicsCalculatorTest extends DRCInverseDynamicsCalculatorTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   
   @Override
   public DRCRobotModel getRobotModel()
   {
      robotModel.setEnableJointDamping(false);
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
