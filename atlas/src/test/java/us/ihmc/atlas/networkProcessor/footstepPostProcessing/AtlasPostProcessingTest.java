package us.ihmc.atlas.networkProcessor.footstepPostProcessing;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasFootstepPlannerParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPostProcessing.AvatarPostProcessingTests;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasPostProcessingTest extends AvatarPostProcessingTests
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               public boolean createFootholdExplorationTools()
               {
                  return false;
               }
            };
         }

         public FootstepPlannerParametersBasics getFootstepPlannerParameters()
         {
            FootstepPlannerParametersBasics plannerParameters = new AtlasFootstepPlannerParameters();
            plannerParameters.setReturnBestEffortPlan(false);
            return plannerParameters;
         }

         public FootstepPostProcessingParametersBasics getFootstepPostProcessingParameters()
         {
            FootstepPostProcessingParametersBasics parametersBasics = new DefaultFootstepPostProcessingParameters();
            parametersBasics.setPositionSplitFractionProcessingEnabled(true);
            parametersBasics.setStepHeightForLargeStepDown(0.05);
            parametersBasics.setLargestStepDownHeight(0.3);
            parametersBasics.setTransferSplitFractionAtFullDepth(0.15);
            parametersBasics.setTransferWeightDistributionAtFullDepth(0.8);
            parametersBasics.setSwingOverRegionsProcessingEnabled(true);
            parametersBasics.setDoInitialFastApproximation(true);
            parametersBasics.setMinimumSwingFootClearance(0.0);
            parametersBasics.setNumberOfChecksPerSwing(100);
            parametersBasics.setMaximumNumberOfAdjustmentAttempts(50);
            parametersBasics.setMaximumWaypointAdjustmentDistance(0.2);
            parametersBasics.setIncrementalWaypointAdjustmentDistance(0.03);
            parametersBasics.setMinimumHeightAboveFloorForCollision(0.03);

            return parametersBasics;
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
