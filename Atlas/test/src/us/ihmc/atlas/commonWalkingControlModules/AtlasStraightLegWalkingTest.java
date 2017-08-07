package us.ihmc.atlas.commonWalkingControlModules;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.util.ArrayList;
import java.util.List;

public class AtlasStraightLegWalkingTest extends AvatarStraightLegWalkingTest
{
   private final AtlasRobotModel atlasRobotModel = new MyAtlasRobotModel();

   @ContinuousIntegrationTest(estimatedDuration =  20.0, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 120000)
   public void testForwardWalking() throws SimulationExceededMaximumTimeException
   {
      try
      {
         super.testForwardWalking();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 120000)
   public void testWalkingOverCinderBlockField() throws Exception
   {
      try
      {
         super.testWalkingOverCinderBlockField();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   private class MyAtlasRobotModel extends AtlasRobotModel
   {
      public MyAtlasRobotModel()
      {
         super(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
         {
            /*
            @Override
            public double getDefaultTransferTime()
            {
               return 0.15;
            }

            @Override
            public double getDefaultSwingTime()
            {
               return 0.9 - getDefaultTransferTime();
            }
            */

            @Override
            public double getMaximumToeOffAngle()
            {
               return Math.toRadians(20);
            }

            @Override
            public boolean checkCoPLocationToTriggerToeOff()
            {
               return true;
            }

            @Override
            public double getCoPProximityForToeOff()
            {
               return 0.05;
            }

            @Override
            public boolean doHeelTouchdownIfPossible()
            {
               return true;
            }

            @Override
            public double getHeelTouchdownLengthRatio()
            {
               return 0.5;
            }

            @Override
            public boolean doToeOffIfPossibleInSingleSupport()
            {
               return true;
            }

            @Override
            public double getAnkleLowerLimitToTriggerToeOff()
            {
               return -0.75;
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
               return 0.08;
            }

            @Override
            public boolean checkECMPLocationToTriggerToeOff()
            {
               return true;
            }

            @Override
            public double getECMPProximityForToeOff()
            {
               return 0.02;
            }

            @Override
            public boolean useOptimizationBasedICPController()
            {
               return true;
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
            public boolean useSingularityAvoidanceInSwing()
            {
               return false;
            }

            @Override
            public boolean useSingularityAvoidanceInSupport()
            {
               return false;
            }


            @Override
            public LeapOfFaithParameters getLeapOfFaithParameters()
            {
               return new LeapOfFaithParameters()
               {
                  @Override
                  public boolean scaleFootWeight()
                  {
                     return true;
                  }

                  @Override
                  public boolean usePelvisRotation()
                  {
                     return true;
                  }

                  @Override
                  public boolean relaxPelvisControl()
                  {
                     return false;
                  }
               };
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
                     return 1.5;
                  }

                  @Override
                  public double getPrivilegedMaxVelocity()
                  {
                     return super.getPrivilegedMaxVelocity();
                  }

                  @Override
                  public boolean attemptToStraightenLegs()
                  {
                     return true;
                  }

                  @Override
                  public double getLegPitchPrivilegedWeight()
                  {
                     return 10.0;
                  }

                  @Override
                  public double getKneeStraightLegPrivilegedWeight()
                  {
                     return 200.0;
                  }

                  @Override
                  public double getKneeBentLegPrivilegedWeight()
                  {
                     return 10.0;
                  }
               };
            }

            @Override
            public MomentumOptimizationSettings getMomentumOptimizationSettings()
            {
               return new AtlasMomentumOptimizationSettings(getJointMap(), getContactPointParameters().getNumberOfContactableBodies())
               {
                  @Override
                  public double getJointAccelerationWeight()
                  {
                     //return 0.005;
                     return 0.05;
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
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new AtlasContinuousCMPPlannerParameters(getPhysicalProperties())
         {
            @Override
            public double getMinTimeToSpendOnExitCoPInSingleSupport()
            {
               return 0.0;//0.05; //0.15;
            }

            @Override
            public double getExitCoPForwardSafetyMarginOnToes()
            {
               return 0.002;
            }

            @Override
            public boolean putExitCoPOnToes()
            {
               return true;
            }

            /** {@inheritDoc} */
            @Override
            public List<Vector2D> getCoPOffsets()
            {
               Vector2D entryOffset = new Vector2D(0.0, -0.005);
               Vector2D exitOffset = new Vector2D(0.0, 0.015);

               List<Vector2D> copOffsets = new ArrayList<>();
               copOffsets.add(entryOffset);
               copOffsets.add(exitOffset);

               return copOffsets;
            }
         };
      }
   }

   public static void main(String[] args) throws Exception
   {
      AtlasStraightLegWalkingTest test = new AtlasStraightLegWalkingTest();
      test.testForwardWalking();
   }
}
