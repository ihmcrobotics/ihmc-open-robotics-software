package us.ihmc.atlas.commonWalkingControlModules;

import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.NewRobotPhysicalProperties;
import us.ihmc.commonWalkingControlModules.AvatarStraightLegWalkingTest;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
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
      super.testForwardWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 120000)
   public void testWalkingOverCinderBlockField() throws Exception
   {
      super.testWalkingOverCinderBlockField();
   }

   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 120000)
   public void testDropOffsWhileWalking() throws Exception
   {
      double stepDownHeight = 0.08;
      super.testDropOffsWhileWalking(stepDownHeight);
   }

   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 120000)
   public void testSteppingDown() throws Exception
   {
      double stepDownHeight = 0.15;
      super.testSteppingDown(stepDownHeight, 0.30, 1);
   }

   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 120000)
   public void testSteppingDownEveryTime() throws Exception
   {
      double stepLength = 0.35;
      double stepDownHeight = 0.15;
      super.testSteppingDown(stepDownHeight, stepLength, 0);
   }

   @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 120000)
   public void testRandomHeightField() throws Exception
   {
      double maxStepIncrease = 0.07;
      double maxStepHeight = 0.04;
      double minStepHeight = -0.12;
      super.testRandomHeightField(maxStepHeight, minStepHeight, maxStepIncrease);
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
         super(DRCRobotModel.RobotTarget.SCS, jointMap, contactPointParameters);

         this.jointMap = jointMap;
         this.contactPointParameters = contactPointParameters;
      }

      @Override
      public boolean rampUpAllowableToeLoadAfterContact()
      {
         return true;
      }

      @Override
      public double getToeLoadingDuration()
      {
         return 0.2;
      }

      @Override
      public boolean controlHeightWithMomentum()
      {
         return false;
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
      public LeapOfFaithParameters getLeapOfFaithParameters()
      {
         return new TestLeapOfFaithParameters();
      }

      @Override
      public StraightLegWalkingParameters getStraightLegWalkingParameters()
      {
         return new TestStraightLegWalkingParameters();
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return new TestMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
      }

      @Override
      public PelvisOffsetWhileWalkingParameters getPelvisOffsetWhileWalkingParameters()
      {
         return new TestPelvisOffsetWhileWalkingParameters();
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
      public boolean doToeOffIfPossibleInSingleSupport()
      {
         return true;
      }

      @Override
      public double getAnkleLowerLimitToTriggerToeOff()
      {
         return -0.75;
      }

   }

   private class TestSwingTrajectoryParameters extends AtlasSwingTrajectoryParameters
   {
      public TestSwingTrajectoryParameters()
      {
         super(DRCRobotModel.RobotTarget.SCS, 1.0);
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
         return true;
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
         return true;
      }
   }

   private class TestStraightLegWalkingParameters extends AtlasStraightLegWalkingParameters
   {
      public TestStraightLegWalkingParameters()
      {
         super(false);
      }

      @Override
      public boolean attemptToStraightenLegs()
      {
         return true;
      }

      @Override
      public LegConfigurationGains getBentLegGains()
      {
         LegConfigurationGains gains = new LegConfigurationGains();
         gains.setJointSpaceKp(100.0);
         gains.setJointSpaceKd(6.0);

         return gains;
      }

      @Override
      public double getLegPitchPrivilegedWeight()
      {
         return 1.0;
      }

      @Override
      public double getLegPrivilegedLowWeight()
      {
         return 0.5;
      }

      @Override
      public double getLegPrivilegedHighWeight()
      {
         return 150.0;
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

   private class TestPelvisOffsetWhileWalkingParameters extends PelvisOffsetWhileWalkingParameters
   {
      @Override
      public boolean addPelvisOrientationOffsetsFromWalkingMotion()
      {
         return true;
      }
   }

   private class TestICPPlannerParameters extends AtlasContinuousCMPPlannerParameters
   {
      public TestICPPlannerParameters(AtlasPhysicalProperties physicalProperties)
      {
         super(physicalProperties);
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
   }

   public static void main(String[] args) throws Exception
   {
      AtlasStraightLegWalkingTest test = new AtlasStraightLegWalkingTest();
      test.testForwardWalking();
   }
}
