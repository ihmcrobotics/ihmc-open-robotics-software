package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryATest;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieICPOptimizationParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieICPOptimizationPushRecoveryATest extends AvatarICPOptimizationPushRecoveryATest
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), RobotTarget.SCS)
            {
               @Override
               public ICPOptimizationParameters getICPOptimizationParameters()
               {
                  return new ValkyrieICPOptimizationParameters(RobotTarget.SCS)
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
                  };
               }
            };
         }
      };

      return valkyrieRobotModel;
   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public double getSlowSwingDuration()
   {
      return 1.2;
   }

   @Override
   public double getSlowTransferDuration()
   {
      return 0.8;
   }

   @Override
   @Test
   public void testPushICPOptimizationBackwardPushInSwing() throws Exception
   {
      percentWeight = 0.2;
      super.testPushICPOptimizationBackwardPushInSwing();
   }

   @Override
   @Test
   public void testPushICPOptimizationForwardPushInSlowSwing() throws Exception
   {
      percentWeight = 0.2;
      super.testPushICPOptimizationForwardPushInSlowSwing();
   }

   @Override
   @Test
   public void testPushICPOptimizationForwardPushInSwing() throws Exception
   {
      percentWeight = 0.29;
      super.testPushICPOptimizationForwardPushInSwing();
   }

   @Override
   @Test
   public void testPushICPOptimizationInwardPushInSwing() throws Exception
   {
      percentWeight = 0.17;
      super.testPushICPOptimizationInwardPushInSwing();
   }

   @Override
   @Test
   public void testPushICPOptimizationLongInwardPushInSwing() throws Exception
   {
      percentWeight = 0.05;
      super.testPushICPOptimizationLongInwardPushInSwing();
   }

   @Override
   @Test
   public void testPushICPOptimizationOutwardPushInTransfer() throws Exception
   {
      percentWeight = 0.12;
      super.testPushICPOptimizationOutwardPushInTransfer();
   }

   @Override
   @Test
   public void testPushICPOptimizationOutwardPushOnEachStep() throws Exception
   {
      percentWeight = 0.12;
      super.testPushICPOptimizationOutwardPushOnEachStep();
   }

   public static void main(String[] args)
   {
      ValkyrieICPOptimizationPushRecoveryATest test = new ValkyrieICPOptimizationPushRecoveryATest();
      try
      {
         test.testPushICPOptimizationOutwardPushOnEachStep();
      }
      catch (Exception e)
      {

      }
   }
}
