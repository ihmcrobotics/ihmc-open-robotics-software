package us.ihmc.atlas;

import org.junit.Test;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarAngularMomentumWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoBoolean;

import static org.junit.Assert.assertTrue;

public class AtlasAngularMomentumWalkingTest extends AvatarAngularMomentumWalkingTest
{
   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private final AtlasJointMap jointMap = new AtlasJointMap(version, new AtlasPhysicalProperties());
   private final RobotTarget target = RobotTarget.SCS;
   private final AtlasRobotModel robotModel = new AtlasRobotModel(version, target, false)
   {
      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new AtlasSmoothCMPPlannerParameters(new AtlasPhysicalProperties());
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasWalkingControllerParameters(target, jointMap, getContactPointParameters())
         {
            @Override
            public boolean alwaysAllowMomentum()
            {
               return true;
            }

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
               };
            }
         };
      }

   };


   @Override
   @ContinuousIntegrationTest(estimatedDuration = 57.4)
   @Test(timeout = 290000)
   public void testForwardWalk() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalk();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testForwardWalkWithCorruptedMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkWithCorruptedMomentum();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testForwardWalkTransferDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkTransferDelayedMomentum();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testForwardWalkTransferBigDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkTransferBigDelayedMomentum();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testForwardWalkSwingDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkSwingDelayedMomentum();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testForwardWalkZeroMomentumFirstStep() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkZeroMomentumFirstStep();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testForwardWalkNoMomentumFirstStep() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkNoMomentumFirstStep();
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

}
