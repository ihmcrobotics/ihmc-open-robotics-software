package us.ihmc.openAlexander;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
import us.ihmc.avatar.AvatarPauseWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class AlexanderPauseWalkingTest extends AvatarPauseWalkingTest
{
   private final RobotTarget target = RobotTarget.SCS;

   private final OpenAlexanderRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, target)
   {
      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new OpenAlexanderWalkingControllerParameters(getRobotVersion(), target, getJointMap(), getPhysicalProperties(), getContactPointParameters())
         {
            @Override
            public double getDefaultFinalTransferTime()
            {
               return getFinalTransferDuration();
            }
         };
      }
   };

   @Tag("allocation-slow")
   @Test
   @Override
   public void testPauseWalking()
   {
      super.testPauseWalking();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testTwoIndependentSteps()
   {
      super.testTwoIndependentSteps();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testStartSecondStepWhileTransitioningToStand()
   {
      super.testStartSecondStepWhileTransitioningToStand();
   }

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testPauseWalkingForward()
   {
      super.testPauseWalkingForward();
   }

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testPauseWalkingInitialTransfer()
   {
      super.testPauseWalkingInitialTransfer();
   }

   @Tag("humanoid-flat-ground-slow-2")
   @Test
   @Override
   public void testPauseWalkingInitialTransferOneStep()
   {
      super.testPauseWalkingInitialTransferOneStep();
   }

   @Tag("allocation-slow")
   @Test
   @Override
   public void testPauseWalkingForwardInitialTransfer()
   {
      super.testPauseWalkingForwardInitialTransfer();
   }

   @Override
   public double getMaxICPPlanError()
   {
      return 0.02;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public double getSwingTime()
   {
      return 1.2;
   }

   @Override
   public double getTransferTime()
   {
      return 0.8;
   }

   @Override
   public double getFinalTransferDuration()
   {
      return 1.5;
   }

   @Override
   public double getStepLength()
   {
      return 0.3;
   }

   @Override
   public double getStepWidth()
   {
      return 0.25;
   }

   @Override
   public double getTimeForPausing()
   {
      return 2.55;
   }

   @Override
   public double getTimeForResuming()
   {
      return 2.0;
   }

   @Override
   public int getNumberOfFootsteps()
   {
      return 5;
   }
}
