package us.ihmc.atlas.networkProcessor.footstepPostProcessing;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPostProcessing.AvatarPostProcessingTests;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

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

   @Tag("humanoid-obstacle-2")
   @Override
   @Test
   public void testWalkingOffOfMediumPlatform()
   {
      super.testWalkingOffOfMediumPlatform();
   }

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
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }
}
