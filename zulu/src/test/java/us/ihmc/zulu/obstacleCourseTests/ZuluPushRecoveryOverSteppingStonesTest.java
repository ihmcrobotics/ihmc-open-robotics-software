package us.ihmc.zulu.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarPushRecoveryOverSteppingStonesTest;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.zulu.parameters.controller.ZuluStepAdjustmentParameters;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;

@Tag("humanoid-obstacle-slow-3")
public class ZuluPushRecoveryOverSteppingStonesTest extends AvatarPushRecoveryOverSteppingStonesTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      ZuluRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ZuluWalkingControllerParameters(getRobotVersion(), RobotTarget.SCS, getJointMap(), getPhysicalProperties(), getContactPointParameters())
            {
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

               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new SteppingParameters()
                  {
                     @Override
                     public double getMaxStepLength()
                     {
                        return 0.8;
                     }

                     @Override
                     public double getMaxStepWidth()
                     {
                        return 0.65;
                     }
                  };
               }

               @Override
               public ToeOffParameters getToeOffParameters()
               {
                  return new ToeOffParameters()
                  {
                     @Override
                     public double getAnkleLowerLimitToTriggerToeOff()
                     {
                        return -0.9;
                     }
                  };
               }
            };
         }
      };

      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Override
   @Test
   public void testWalkingOverSteppingStonesForwardPush()
   {
      super.testWalkingOverSteppingStonesForwardPush();
   }
}
