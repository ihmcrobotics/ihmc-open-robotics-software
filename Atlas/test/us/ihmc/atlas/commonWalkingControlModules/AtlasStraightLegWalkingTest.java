package us.ihmc.atlas.commonWalkingControlModules;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
               public YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry)
               {
                  YoFootSE3Gains gains = new YoFootSE3Gains("SwingFoot", registry);

                  double kpXY = 150.0;
                  double kpZ = 200.0; //200.0;
                  double zetaXYZ = 0.7;

                  double kpXYOrientation = 200.0; // 200.0;
                  double kpZOrientation = 200.0; // 200.0;
                  double zetaOrientation = 0.7;

                  double maxPositionAcceleration = Double.POSITIVE_INFINITY;
                  double maxPositionJerk = Double.POSITIVE_INFINITY;
                  double maxOrientationAcceleration = Double.POSITIVE_INFINITY;
                  double maxOrientationJerk = Double.POSITIVE_INFINITY;

                  double kdReductionRatio = 1.0;
                  double parallelDampingDeadband = 100.0;
                  double positionErrorForMinimumKd = 10000.0;

                  gains.setPositionProportionalGains(kpXY, kpZ);
                  gains.setPositionDampingRatio(zetaXYZ);
                  gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
                  gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
                  gains.setOrientationDampingRatio(zetaOrientation);
                  gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);
                  gains.setTangentialDampingGains(kdReductionRatio, parallelDampingDeadband, positionErrorForMinimumKd);
                  gains.createDerivativeGainUpdater(true);

                  return gains;
               }

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
               public boolean applySecondaryJointScaleDuringSwing()
               {
                  return true;
               }

               @Override
               public AtlasStraightLegWalkingParameters getStraightLegWalkingParameters()
               {
                  return new AtlasStraightLegWalkingParameters(false)
                  {
                     @Override
                     public double getSpeedForStanceLegStraightening()
                     {
                        return 1.0;
                     }

                     @Override
                     public double getPercentOfSwingToStraightenLeg()
                     {
                        return 0.8;
                     }

                     @Override
                     public double getPercentOfTransferToCollapseLeg()
                     {
                        return 0.8;
                     }

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

                     @Override
                     public double getStraightLegPrivilegedConfigurationGain()
                     {
                        return 50.0;
                     }

                     @Override
                     public double getStraightLegPrivilegedVelocityGain()
                     {
                        return 4.0; // 6.0;
                     }

                     @Override
                     public double getStraightLegPrivilegedWeight()
                     {
                        return 100.0;
                     }

                     @Override
                     public double getBentLegPrivilegedConfigurationGain()
                     {
                        return 150.0;
                     }

                     @Override
                     public double getBentLegPrivilegedVelocityGain()
                     {
                        return 4.0;
                     }

                     @Override
                     public double getBentLegPrivilegedWeight()
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
