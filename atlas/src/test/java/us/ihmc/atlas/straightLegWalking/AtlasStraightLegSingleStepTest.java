package us.ihmc.atlas.straightLegWalking;

import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegSingleStepTest;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.util.EnumMap;

public class AtlasStraightLegSingleStepTest extends AvatarStraightLegSingleStepTest
{
   private final AtlasRobotModel atlasRobotModel = new MyAtlasRobotModel();

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 70000)
   public void testForwardStep() throws SimulationExceededMaximumTimeException
   {
      double stepLength = 1.5;
      double stepWidth = 0.25;

      setStepLength(stepLength);
      setStepWidth(stepWidth);

      super.testForwardStep();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 70000)
   public void testForwardStepWithPause() throws SimulationExceededMaximumTimeException
   {
      double stepLength = 1.0;
      double stepWidth = 0.25;

      setStepLength(stepLength);
      setStepWidth(stepWidth);

      super.testForwardStepWithPause();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 99990000)
   public void testForwardSteps() throws SimulationExceededMaximumTimeException
   {
      super.testForwardSteps();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 70000)
   public void testWideStep() throws SimulationExceededMaximumTimeException
   {
      double stepWidth = 0.6;
      double stanceWidth = 0.25;

      setStepWidth(stepWidth);
      setStanceWidth(stanceWidth);

      super.testWideStep();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 100000)
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

   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 100000)
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
      public boolean controlHeightWithMomentum()
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
         return true;
      }

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
