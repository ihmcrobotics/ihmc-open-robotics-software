package us.ihmc.atlas.commonWalkingControlModules;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
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
               public double getMaxICPErrorBeforeSingleSupportX()
               {
                  return 0.05;
               }

               @Override
               public double getMaxICPErrorBeforeSingleSupportY()
               {
                  return 0.03;
               }

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
               public boolean checkECMPLocationToTriggerToeOff()
               {
                  return true;
               }

               @Override
               public double getECMPProximityForToeOff()
               {
                  return 0.03;
               }

               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true; //false;
               }

               @Override
               public boolean editStepTimingForReachability()
               {
                  return false; // // TODO: 4/27/17  
               }

               @Override
               public boolean applySecondaryJointScaleDuringSwing()
               {
                  return true;
               }

               @Override
               public double[] getSwingWaypointProportions()
               {
                  return new double[] {0.15, 0.80};
               }

               @Override
               public AtlasStraightLegWalkingParameters getStraightLegWalkingParameters()
               {
                  return new AtlasStraightLegWalkingParameters(false)
                  {
                     @Override
                     public double getSpeedForSupportKneeStraightening()
                     {
                        return 1.0;
                     }

                     @Override
                     public double getFractionOfSwingToStraightenLeg()
                     {
                        return 1.0;//0.75;
                     }

                     @Override
                     public double getFractionOfTransferToCollapseLeg()
                     {
                        return 0.6;
                     }

                     @Override
                     public double getFractionOfSwingToCollapseStanceLeg()
                     {
                        return 1.0;
                     }

                     @Override
                     public boolean attemptToStraightenLegs()
                     {
                        return true;
                     }

                     @Override
                     public double getStraightKneeAngle()
                     {
                        return 0.2;
                     }

                     @Override
                     public double getLegPitchPrivilegedWeight()
                     {
                        return 10.0;
                     }

                     @Override
                     public double getKneeStraightLegPrivilegedConfigurationGain()
                     {
                        return 50.0;
                     }

                     @Override
                     public double getKneeStraightLegPrivilegedVelocityGain()
                     {
                        return 4.0; // 6.0;
                     }

                     @Override
                     public double getKneeStraightLegPrivilegedWeight()
                     {
                        return 200.0;
                     }

                     @Override
                     public double getKneeBentLegPrivilegedConfigurationGain()
                     {
                        return 150.0;
                     }

                     @Override
                     public double getKneeBentLegPrivilegedVelocityGain()
                     {
                        return 4.0;
                     }

                     @Override
                     public double getKneeBentLegPrivilegedWeight()
                     {
                        return 10.0;
                     }

                     @Override
                     public double getPrivilegedMaxAcceleration()
                     {
                        return 200.0;
                     }
                  };
               }

               @Override
               public MomentumOptimizationSettings getMomentumOptimizationSettings()
               {
                  return new AtlasMomentumOptimizationSettings(getJointMap(), getContactPointParameters().getNumberOfContactableBodies())
                  {
                     @Override
                     public Vector3D getHighAngularFootWeight()
                     {
                        return new Vector3D(5.0, 5.0, 5.0);
                     }

                     @Override
                     public Vector3D getDefaultAngularFootWeight()
                     {
                        return new Vector3D(0.1, 0.1, 0.1);// Vector3D(0.5, 0.5, 0.5);
                     }

                     @Override
                     public Vector3D getDefaultLinearFootWeight()
                     {
                        return new Vector3D(10.0, 10.0, 10.0);// Vector3D(30.0, 30.0, 30.0);
                     }

                     @Override
                     public double getJointAccelerationWeight()
                     {
                        return 0.05; //0.005;
                     }

                     @Override
                     public double getJointJerkWeight()
                     {
                        return 0.5; //0.1;
                     }

                     @Override
                     public double getRhoRateDefaultWeight()
                     {
                        return 0.005; //0.002;
                     }
                  };
               }

               @Override
               public PelvisOffsetWhileWalkingParameters getPelvisOffsetWhileWalkingParameters()
               {
                  return new PelvisOffsetWhileWalkingParameters()
                  {
                     @Override
                     public boolean addPelvisOrientationOffsetsFromWalkingMotion()
                     {
                        return true;
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
                  return 0.1;
               }

               @Override
               public double getExitCMPForwardSafetyMarginOnToes()
               {
                  return 0.0;
               }

               @Override
               public boolean putExitCMPOnToes()
               {
                  return true;
               }

               @Override
               public double getExitCMPInsideOffset()
               {
                  return 0.015;
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
