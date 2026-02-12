package us.ihmc.zulu;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.avatar.AvatarStepInPlaceTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class ZuluStepInPlaceTest extends AvatarStepInPlaceTest
{
   private final ZuluVersion version = ZuluVersion.V1_FULL_ROBOT;
   private final RobotTarget target = RobotTarget.SCS;
   private final ZuluRobotModel robotModel = new ZuluRobotModel(version, target)
   {
      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new ZuluWalkingControllerParameters(getRobotVersion(), target, getJointMap(), getPhysicalProperties(), getContactPointParameters())
         {
            @Override
            public StepAdjustmentParameters getStepAdjustmentParameters()
            {
               return new StepAdjustmentParameters()
               {
                  @Override
                  public double getMinICPErrorForStepAdjustment()
                  {
                     return 0.04;
                  }
               };
            }
         };
      }
   };

   private final int numberOfSteps = 1;
   private final double stepWidth = 0.0;
   private final double stepLength = 0.5;

   @Tag("humanoid-flat-ground-slow")
   @Override
   public void testStepInPlace()
   {
      super.testStepInPlace();
   }

   @Tag("humanoid-flat-ground-slow")
   @Test
   @Override
   public void testStepInPlaceWithPush()
   {
      super.testStepInPlaceWithPush();
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
   public int getNumberOfSteps()
   {
      return numberOfSteps;
   }

   @Override
   public double getStepWidth()
   {
      return stepWidth;
   }

   @Override
   public double getStepLength()
   {
      return stepLength;
   }

}
