package us.ihmc.atlas.commonWalkingControlModules;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasCapturePointPlannerParameters;
import us.ihmc.atlas.parameters.AtlasStraightLegWalkingParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasStraightLegWalkingTest extends AvatarStraightLegWalkingTest
{
   protected DRCRobotModel getRobotModel()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean doToeOffIfPossibleInSingleSupport()
               {
                  return true;
               }

               @Override
               public double getAnkleLowerLimitToTriggerToeOff()
               {
                  return -0.45;
               }

               @Override
               public boolean controlHeightWithMomentum()
               {
                  return false;
               }

               @Override
               public double getICPPercentOfStanceForDSToeOff()
               {
                  return 0.3;
               }

               @Override
               public double getICPPercentOfStanceForSSToeOff()
               {
                  return 0.15;
               }

               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true; // // TODO: 4/4/17 false? 
               }

               @Override
               public AtlasStraightLegWalkingParameters getStraightLegWalkingParameters()
               {
                  return new AtlasStraightLegWalkingParameters()
                  {
                     @Override
                     public boolean attemptToStraightenLegs()
                     {
                        return true;
                     }

                     @Override
                     public double getStraightKneeAngle()
                     {
                        return 0.4;
                     }
                  };
               }

            };
         }

         @Override
         public CapturePointPlannerParameters getCapturePointPlannerParameters()
         {
            return new AtlasCapturePointPlannerParameters(getPhysicalProperties())
            {
               @Override
               public double getMinTimeToSpendOnExitCMPInSingleSupport()
               {
                  return 0.2;
               }

               @Override
               public boolean putExitCMPOnToes()
               {
                  return true;
               }
            };
         }
      };

      return atlasRobotModel;
   }

   public static void main(String[] args)
   {
      AtlasStraightLegWalkingTest test = new AtlasStraightLegWalkingTest();
      try
      {
         test.testForwardWalking();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }
}
