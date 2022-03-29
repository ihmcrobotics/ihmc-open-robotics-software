package us.ihmc.atlas.roughTerrainWalking;

import java.io.InputStream;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPControllerParameters;
import us.ihmc.atlas.parameters.AtlasStepAdjustmentParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarPushRecoveryOverCinderBlocksTest;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushRecoveryOverCinderBlocksTest extends AvatarPushRecoveryOverCinderBlocksTest
{
   @Override
   @Tag("humanoid-rough-terrain-2")
   @Disabled
   @Test
   public void testNoPushFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushFlatBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testNoPushForwardWalkOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushForwardWalkOverFlatBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testNoPushTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushTiltedBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testNoPushForwardTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testNoPushForwardTiltedBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testPushOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testPushOverFlatBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testForwardPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testForwardPushWalkWithOffsetOverFlatBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testLeftSidewaysPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testLeftSidewaysPushWalkWithOffsetOverFlatBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
   public void testRightSidewaysPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      super.testRightSidewaysPushWalkWithOffsetOverFlatBlocks();
   }

   @Override
   @Tag("humanoid-rough-terrain")
   @Disabled
   @Test
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
               public ICPControllerParameters getICPControllerParameters()
               {
                  return new AtlasICPControllerParameters(false)
                  {
                     @Override
                     public boolean useAngularMomentum()
                     {
                        return true;
                     }
                  };
               }

               @Override
               public StepAdjustmentParameters getStepAdjustmentParameters()
               {
                  return new AtlasStepAdjustmentParameters()
                  {
                     @Override
                     public boolean allowStepAdjustment()
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
