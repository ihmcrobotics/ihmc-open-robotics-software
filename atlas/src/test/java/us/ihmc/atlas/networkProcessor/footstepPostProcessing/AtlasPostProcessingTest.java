package us.ihmc.atlas.networkProcessor.footstepPostProcessing;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPostProcessing.AvatarPostProcessingTests;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
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
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
