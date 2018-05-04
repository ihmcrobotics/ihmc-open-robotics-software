package us.ihmc.atlas.roughTerrainWalking;

import java.io.InputStream;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarPushRecoveryOverCinderBlocksTest;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasPushRecoveryOverCinderBlocksTest extends AvatarPushRecoveryOverCinderBlocksTest
{
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 72.3)
   @Test(timeout = 360000)
   public void testNoPushFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushFlatBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 64.7)
   @Test(timeout = 320000)
   public void testNoPushForwardWalkOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushForwardWalkOverFlatBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 94.5)
   @Test(timeout = 470000)
   public void testNoPushTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushTiltedBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 68.6)
   @Test(timeout = 340000)
   public void testNoPushForwardTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushForwardTiltedBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 180000)
   public void testPushOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testPushOverFlatBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 180000)
   public void testForwardPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testForwardPushWalkWithOffsetOverFlatBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 180000)
   public void testLeftSidewaysPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testLeftSidewaysPushWalkWithOffsetOverFlatBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 180000)
   public void testRightSidewaysPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testRightSidewaysPushWalkWithOffsetOverFlatBlocks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 180000)
   public void testPushOverTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testPushOverTiltedBlocks();
   }


   @Override
   public DRCRobotModel getRobotModel()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true;
               }

               @Override
               public double getMinimumSwingTimeForDisturbanceRecovery()
               {
                  return 0.5;
               }

               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new AtlasSteppingParameters(getJointMap())
                  {
                     @Override
                     public double getMaxStepLength()
                     {
                        return 1.2;
                     }
                  };
               }

               @Override
               public ICPOptimizationParameters getICPOptimizationParameters()
               {
                  return new AtlasICPOptimizationParameters(false)
                  {
                     @Override
                     public boolean useAngularMomentum()
                     {
                        return true;
                     }

                     @Override
                     public boolean allowStepAdjustment()
                     {
                        return true;
                     }

                     @Override
                     public boolean usePlanarRegionConstraints()
                     {
                        return true;
                     }

                     @Override
                     public boolean switchPlanarRegionConstraintsAutomatically()
                     {
                        return true;
                     }
                  };
               }
            };
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return getClass().getClassLoader().getResourceAsStream("atlasPushRecoveryOverCinderBlocksTest.xml");
         }
      };

      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
