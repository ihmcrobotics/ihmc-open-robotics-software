package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidPointyRocksEnvironmentContactsTest;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasPointyRocksEnvironmentContactsTest extends HumanoidPointyRocksEnvironmentContactsTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return null;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected DRCRobotModel getRobotModel(int xContactPoints, int yContactPoints, boolean createOnlyEdgePoints)
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, xContactPoints, yContactPoints, createOnlyEdgePoints, true);
      AtlasRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
      return robotModel;
   }

   @Override
   @Disabled
   @Test
   public void testWalkingOnLinesInEnvironment() throws SimulationExceededMaximumTimeException
   {
      try
      {
         super.testWalkingOnLinesInEnvironment();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         throw e;
      }
      catch (AssertionError e)
      {
         e.printStackTrace();
         throw e;
      }
   }

   @Override
   @Disabled
   @Test
   public void testWalkingOnPointInEnvironment() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnPointInEnvironment();
   }

   private class TestModel extends AtlasRobotModel
   {
      private final TestWalkingParameters walkingParameters;

      public TestModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless, FootContactPoints<RobotSide> simulationContactPoints)
      {
         super(atlasVersion, target, headless, simulationContactPoints);
         walkingParameters = new TestWalkingParameters(target, getJointMap(), getContactPointParameters());
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return walkingParameters;
      }
   }

   private class TestWalkingParameters extends AtlasWalkingControllerParameters
   {
      private final TestICPOptimizationParameters icpOptimizationParameters;
      private final TestToeOffParameters toeOffParameters;

      public TestWalkingParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);
         icpOptimizationParameters = new TestICPOptimizationParameters();
         toeOffParameters = new TestToeOffParameters(jointMap);
      }

      @Override
      public boolean createFootholdExplorationTools()
      {
         return true;
      }

      @Override
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return icpOptimizationParameters;
      }

      @Override
      public ToeOffParameters getToeOffParameters()
      {
         return toeOffParameters;
      }
   }

   private class TestICPOptimizationParameters extends AtlasICPOptimizationParameters
   {
      public TestICPOptimizationParameters()
      {
         super(false);
      }

      @Override
      public boolean useAngularMomentum()
      {
         return true;
      }
   }

   private class TestToeOffParameters extends AtlasToeOffParameters
   {
      public TestToeOffParameters(AtlasJointMap jointMap)
      {
         super(jointMap);
      }

      @Override
      public boolean doToeOffIfPossible()
      {
         return false;
      }
   }
}
