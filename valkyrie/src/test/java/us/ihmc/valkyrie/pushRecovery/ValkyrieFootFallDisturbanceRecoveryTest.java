package us.ihmc.valkyrie.pushRecovery;

import java.util.function.Consumer;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.HumanoidFootFallDisturbanceRecoveryTest;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieSwingTrajectoryParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ValkyrieFootFallDisturbanceRecoveryTest extends HumanoidFootFallDisturbanceRecoveryTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), getTarget())
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
                  return new ValkyrieSwingTrajectoryParameters(getRobotPhysicalProperties(), getTarget())
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

   @Disabled // TODO Falls on the way out of the hole because of a poor CMP positioning. 
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

   @Disabled // Disabled as pretty redundant with the 15cm test. 
   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOver_10cm_StepDown(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableOffsetFootstepsHeightWithExecutionError();
      super.testBlindWalkOverStepDown(testInfo, 0.6, 0.25, 0.10);
   }

   @Tag("humanoid-push-recovery-slow")
   @Test
   public void testBlindWalkOver_15cm_StepDown(TestInfo testInfo) throws Exception
   {
      useExperimentalPhysicsEngine();
      enableToeOffInSingleSupport();
      enableOffsetFootstepsHeightWithExecutionError();
      super.testBlindWalkOverStepDown(testInfo, 0.6, 0.25, 0.15);
   }

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
