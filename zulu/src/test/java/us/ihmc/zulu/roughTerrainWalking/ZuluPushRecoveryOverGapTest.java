package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.controller.ZuluStepAdjustmentParameters;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarPushRecoveryOverGapTest;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class ZuluPushRecoveryOverGapTest extends AvatarPushRecoveryOverGapTest
{
   @Override
   @Test
   public void testNoPush()
   {
      super.testNoPush();
   }

   @Override
   @Test
   public void testForwardPush()
   {
      super.testForwardPush();
   }

   @Override
   @Test
   public void testSidePush()
   {
      super.testSidePush();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      ZuluRobotModel zuluRobotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ZuluWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties())
            {
               @Override
               public double getMinimumSwingTimeForDisturbanceRecovery()
               {
                  return 0.45;
               }

               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new SteppingParameters()
                  {
                     @Override
                     public double getMaxStepLength()
                     {
                        return 1.0;
                     }
                  };
               }

               @Override
               public StepAdjustmentParameters getStepAdjustmentParameters()
               {
                  return new ZuluStepAdjustmentParameters()
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

      return zuluRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }
}
