package us.ihmc.atlas.pushRecovery;

import java.util.function.Consumer;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.HumanoidFootFallDisturbanceRecoveryTest;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AtlasFootFallDisturbanceRecoveryTest extends HumanoidFootFallDisturbanceRecoveryTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters())
            {
               @Override
               public double getInitialICPErrorToSlowDownTransfer()
               {
                  return 0.10;
               }

               @Override
               public double getSwingTimeOverrunToInitializeFreeFall()
               {
                  return 0.0;
               }

               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new AtlasSwingTrajectoryParameters(getTarget(), getJointMap().getModelScale())
                  {
                     @Override
                     public Tuple3DReadOnly getTouchdownVelocityWeight()
                     {
                        return new Vector3D(30.0, 30.0, 30.0);
                     }

                     @Override
                     public double getFinalCoMVelocityInjectionRatio()
                     {
                        return 1.0;
                     }

                     @Override
                     public double getFinalCoMAccelerationInjectionRatio()
                     {
                        return 1.0;
                     }
                  };
               }
            };
         }
      };
      return robotModel;
   }

   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOverHole_10cm(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableOffsetFootstepsHeightWithExecutionError();
      testBlindWalkOverHole(testInfo, 0.6, 0.25, 0.10);
   }

   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOverHole_15cm(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableToeOffInSingleSupport();
      enableOffsetFootstepsHeightWithExecutionError();
      testBlindWalkOverHole(testInfo, 0.6, 0.25, 0.15);
   }

   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOverHole_20cm(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableToeOffInSingleSupport();
      enableOffsetFootstepsHeightWithExecutionError();
      Consumer<YoRegistry> varMutator = rootRegistry -> rootRegistry.findVariable("comHeightFallAccelerationMagnitude").setValueFromDouble(7.0);
      testBlindWalkOverHole(testInfo, 0.6, 0.25, 0.20, varMutator);
   }

   @Disabled // TODO It seems that the joint limits are getting in the way for the step down tests
   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOver_10cm_StepDown(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableOffsetFootstepsHeightWithExecutionError();
      super.testBlindWalkOverStepDown(testInfo, 0.6, 0.25, 0.10);
   }

   @Disabled // TODO It seems that the joint limits are getting in the way for the step down tests
   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOver_15cm_StepDown(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableToeOffInSingleSupport();
      enableOffsetFootstepsHeightWithExecutionError();
      super.testBlindWalkOverStepDown(testInfo, 0.6, 0.25, 0.15);
   }

   @Disabled // TODO It seems that the joint limits are getting in the way for the step down tests
   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOver_20cm_StepDown(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableToeOffInSingleSupport();
      enableOffsetFootstepsHeightWithExecutionError();
      Consumer<YoRegistry> varMutator = rootRegistry -> rootRegistry.findVariable("comHeightFallAccelerationMagnitude").setValueFromDouble(7.0);
      super.testBlindWalkOverStepDown(testInfo, 0.6, 0.25, 0.20, varMutator);
   }
}
