package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasICPControllerParameters;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidLineContactWalkingTest;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-rough-terrain-slow")
public class AtlasLineContactWalkingTest extends HumanoidLineContactWalkingTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Disabled
   @Test
   public void testWalkingOnLines() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnLines();
   }

   @Override
   @Disabled
   @Test
   public void testWalkingOnStraightForwardLines() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnStraightForwardLines();
   }

   @Override
   @Disabled
   @Test
   public void testWalkingOnStraightSidewayLines() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnStraightSidewayLines();
   }

   private class TestModel extends AtlasRobotModel
   {
      private final TestWalkingParameters walkingParameters;

      public TestModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless)
      {
         super(atlasVersion, target, headless);
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
      public ICPControllerParameters getICPControllerParameters()
      {
         return icpOptimizationParameters;
      }

      @Override
      public ToeOffParameters getToeOffParameters()
      {
         return toeOffParameters;
      }
   }

   private class TestICPOptimizationParameters extends AtlasICPControllerParameters
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
