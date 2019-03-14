package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryATest;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class AtlasICPOptimizationPushRecoveryATest extends AvatarICPOptimizationPushRecoveryATest
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
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
                     public boolean useSmartICPIntegrator()
                     {
                        return true;
                     }
                  };
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
            };
         }
      };

      return atlasRobotModel;
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
      AtlasICPOptimizationPushRecoveryATest test = new AtlasICPOptimizationPushRecoveryATest();
      try
      {
         test.testPushICPOptimizationOutwardPushOnEachStep();
      }
      catch (Exception e)
      {

      }
   }
}
