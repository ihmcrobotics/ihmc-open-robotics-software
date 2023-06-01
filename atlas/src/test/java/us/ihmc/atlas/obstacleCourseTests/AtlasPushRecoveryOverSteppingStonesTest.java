package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPControllerParameters;
import us.ihmc.atlas.parameters.AtlasStepAdjustmentParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarPushRecoveryOverSteppingStonesTest;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

@Tag("humanoid-obstacle-slow-3")
public class AtlasPushRecoveryOverSteppingStonesTest extends AvatarPushRecoveryOverSteppingStonesTest
{
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

               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new AtlasSteppingParameters(getJointMap())
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
                  return new AtlasToeOffParameters(getJointMap())
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

      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testWalkingOverSteppingStonesForwardPush()
   {
      super.testWalkingOverSteppingStonesForwardPush();
   }
}
