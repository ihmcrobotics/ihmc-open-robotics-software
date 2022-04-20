package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarPushRecoveryOverGapTest;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-rough-terrain")
public class AtlasPushRecoveryOverGapTest extends AvatarPushRecoveryOverGapTest
{
   @Override
   @Test
   public void testNoPush() throws SimulationExceededMaximumTimeException
   {
      super.testNoPush();
   }

   @Override
   @Test
   public void testForwardPush() throws SimulationExceededMaximumTimeException
   {
      super.testForwardPush();
   }

   @Override
   @Test
   public void testSidePush() throws SimulationExceededMaximumTimeException
   {
      super.testSidePush();
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
                  return 0.55;
               }

               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new AtlasSteppingParameters(getJointMap())
                  {
                     @Override
                     public double getMaxStepLength()
                     {
                        return 1.0;
                     }
                  };
               }

               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new AtlasSwingTrajectoryParameters(getTarget(), 1.0)
                  {
                     @Override
                     public double getSwingFootVelocityAdjustmentDamping()
                     {
                        return 0.8;
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
      };

      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
