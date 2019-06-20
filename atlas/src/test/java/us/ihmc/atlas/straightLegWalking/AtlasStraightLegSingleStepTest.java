package us.ihmc.atlas.straightLegWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasLegConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasMomentumOptimizationSettings;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegSingleStepTest;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasStraightLegSingleStepTest extends AvatarStraightLegSingleStepTest
{
   private final AtlasRobotModel atlasRobotModel = new MyAtlasRobotModel();

   @Override
   @Test
   public void testForwardStep() throws SimulationExceededMaximumTimeException
   {
      double stepLength = 1.3;
      double stepWidth = 0.25 ;

      setStepLength(stepLength);
      setStepWidth(stepWidth);

      super.testForwardStep();
   }

   @Override
   @Test
   public void testForwardStepWithPause() throws SimulationExceededMaximumTimeException
   {
      double stepLength = 0.9;
      double stepWidth = 0.25;

      setStepLength(stepLength);
      setStepWidth(stepWidth);

      super.testForwardStepWithPause();
   }

   @Override
   @Disabled
   @Test
   public void testForwardSteps() throws SimulationExceededMaximumTimeException
   {
      super.testForwardSteps();
   }

   @Override
   @Test
   public void testWideStep() throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.6;
      double stanceWidth = 0.25;

      setStepWidth(stepWidth);
      setStanceWidth(stanceWidth);

      super.testWideStep();
   }

   @Override
   @Test
   public void testSteppingDown() throws SimulationExceededMaximumTimeException
   {
      double stepHeight = 0.4;
      double stepLength = 0.4;
      double stanceWidth = 0.25;
      setStepDownHeight(stepHeight);
      setStepHeight(stepHeight);
      setStepLength(stepLength);
      setStanceWidth(stanceWidth);
      super.testSteppingDown();
   }

   @Override
   @Disabled
   @Test
   public void testSteppingDownWithClosing() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.3;
      double stepLength = 0.4;
      double stanceWidth = 0.25;
      setStepDownHeight(stepDownHeight);
      setStepLength(stepLength);
      setStanceWidth(stanceWidth);
      super.testSteppingDownWithClosing();
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
         return new TestWalkingControllerParameters(getJointMap(), getContactPointParameters());
      }

      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new TestICPPlannerParameters(getPhysicalProperties());
      }
   }

   private class TestWalkingControllerParameters extends AtlasWalkingControllerParameters
   {
      private final AtlasJointMap jointMap;
      private final AtlasContactPointParameters contactPointParameters;

      public TestWalkingControllerParameters(AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(RobotTarget.SCS, jointMap, contactPointParameters);

         this.jointMap = jointMap;
         this.contactPointParameters = contactPointParameters;
      }

      @Override
      public double getMaxAllowedDistanceCMPSupport()
      {
         return 0.10;
      }

      @Override
      public boolean enableHeightFeedbackControl()
      {
         return false;
      }

      @Override
      public boolean applySecondaryJointScaleDuringSwing()
      {
         return false;
      }

      @Override
      public LeapOfFaithParameters getLeapOfFaithParameters()
      {
         return new TestLeapOfFaithParameters();
      }

      @Override
      public LegConfigurationParameters getLegConfigurationParameters()
      {
         return new TestLegConfigurationParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new TestMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new TestSwingTrajectoryParameters();
      }

      @Override
      public TestToeOffParameters getToeOffParameters()
      {
         return new TestToeOffParameters(jointMap);
      }

   }

   private class TestToeOffParameters extends AtlasToeOffParameters
   {
      public TestToeOffParameters(AtlasJointMap jointMap)
      {
         super(jointMap);
      }

      @Override
      public boolean checkCoPLocationToTriggerToeOff()
      {
         return false;
      }

      @Override
      public double getCoPProximityForToeOff()
      {
         return 0.05;
      }

      @Override
      public double getICPPercentOfStanceForDSToeOff()
      {
         return 0.20;
      }

      @Override
      public double getICPPercentOfStanceForSSToeOff()
      {
//         return 0.10;
                  return 0.70;// for big step down
      }

      @Override
      public boolean checkECMPLocationToTriggerToeOff()
      {
         return false;
      }

      @Override
      public double getECMPProximityForToeOff()
      {
         return 0.01;
      }

      @Override
      public boolean doToeOffIfPossibleInSingleSupport()
      {
         return false;
      } // FIXME this is disabled because of a bug in FrameConvexPolygon2D

      @Override
      public boolean doToeOffIfPossible()
      {
         return true;
      }

      @Override
      public double getAnkleLowerLimitToTriggerToeOff()
      {
         return -0.75;
      }

      @Override
      public double getKneeLowerLimitToTriggerToeOff()
      {
         return 0.2;
      }

      @Override
      public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
      {
         return true;
      }

      @Override
      public boolean doToeOffWhenHittingLeadingKneeUpperLimit()
      {
         return true;
      }
   }

   private class TestSwingTrajectoryParameters extends AtlasSwingTrajectoryParameters
   {
      public TestSwingTrajectoryParameters()
      {
         super(RobotTarget.SCS, 1.0);
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
      public boolean doHeelTouchdownIfPossible()
      {
         return true;
      }

      @Override
      public boolean doToeTouchdownIfPossible()
      {
         return false;
      }

      @Override
      public boolean addOrientationMidpointForObstacleClearance()
      {
         return true;
      }
   }

   private class TestLeapOfFaithParameters extends LeapOfFaithParameters
   {
      @Override
      public boolean scaleFootWeight()
      {
         return false;
      }
   }

   private class TestLegConfigurationParameters extends AtlasLegConfigurationParameters
   {
      public TestLegConfigurationParameters()
      {
         super(false);
      }

      @Override
      public boolean attemptToStraightenLegs()
      {
         return true;
      }
   }

   private class TestMomentumOptimizationSettings extends AtlasMomentumOptimizationSettings
   {
      public TestMomentumOptimizationSettings(AtlasJointMap jointMap, int numberOfContactableBodies)
      {
         super(jointMap, numberOfContactableBodies);
      }

      @Override
      public double getJointAccelerationWeight()
      {
         return 0.05;
      }

      @Override
      public double getRhoRateDefaultWeight()
      {
         return 5e-7;
      }

      @Override
      public double getJointJerkWeight()
      {
         return 1E-6;
      }
   }

   private class TestICPPlannerParameters extends AtlasSmoothCMPPlannerParameters
   {
      public TestICPPlannerParameters(AtlasPhysicalProperties physicalProperties)
      {
         super(physicalProperties);
      }

      @Override
      public double getTransferSplitFraction()
      {
         return 0.9;
      }

      @Override
      public double getExitCoPForwardSafetyMarginOnToes()
      {
         return 0.02;
      }

      @Override
      public boolean putExitCoPOnToes()
      {
         return true;
      }
   }
}
