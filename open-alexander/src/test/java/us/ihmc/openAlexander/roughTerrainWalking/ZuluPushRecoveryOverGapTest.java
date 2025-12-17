package us.ihmc.openAlexander.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.OpenAlexanderVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderStepAdjustmentParameters;
import us.ihmc.openAlexander.parameters.controller.ZuluAlexanderSteppingParameters;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
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
      OpenAlexanderRobotModel alexanderRobotModel = new OpenAlexanderRobotModel(OpenAlexanderVersion.V1_FULL_ROBOT, RobotTarget.SCS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new OpenAlexanderWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties())
            {
               @Override
               public double getMinimumSwingTimeForDisturbanceRecovery()
               {
                  return 0.45;
               }

               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new ZuluAlexanderSteppingParameters(getPhysicalProperties())
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
                  return new OpenAlexanderStepAdjustmentParameters()
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

      return alexanderRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }
}
