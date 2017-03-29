package us.ihmc.atlas.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasMomentumOptimizationSettings;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.PointFeedbackControllerTest;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasPointFeedbackControllerTest extends PointFeedbackControllerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testConvergence() throws Exception
   {
      super.testConvergence();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testConvergenceWithJerryQP() throws Exception
   {
      super.testConvergenceWithJerryQP();
   }

   @Override
   protected MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
      return new AtlasMomentumOptimizationSettings(robotModel.getJointMap(), robotModel.getContactPointParameters().getNumberOfContactableBodies());
   }
}
