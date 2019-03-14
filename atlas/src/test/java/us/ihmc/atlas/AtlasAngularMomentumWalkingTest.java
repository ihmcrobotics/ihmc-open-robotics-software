package us.ihmc.atlas;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.angularMomentumTest.AvatarAngularMomentumWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

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
   protected double getStepLength()
   {
      return 0.4;
   }

   @Override
   protected double getStepWidth()
   {
      return 0.25;
   }

   @Override
   @Test
   public void testForwardWalkWithAngularMomentumReference() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkWithAngularMomentumReference();
   }

   @Override
   @Test
   public void testForwardWalkWithCorruptedMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkWithCorruptedMomentum();
   }

   @Override
   @Test
   public void testWalkingWithDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithDelayedMomentum();
   }

   @Override
   @Test
   public void testForwardWalkZeroMomentumFirstStep() throws SimulationExceededMaximumTimeException
   {
      super.testForwardWalkZeroMomentumFirstStep();
   }

   @Override
   @Test
   public void testWalkingWithRandomSinusoidalMomentum() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithRandomSinusoidalMomentum();
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
