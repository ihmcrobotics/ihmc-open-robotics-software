package us.ihmc.atlas;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarStepInPlaceTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasStepInPlaceTest extends AvatarStepInPlaceTest
{
   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private final RobotTarget target = RobotTarget.SCS;
   private final AtlasRobotModel robotModel = new AtlasRobotModel(version, target, false)
   {
      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasWalkingControllerParameters(target, getJointMap(), getContactPointParameters())
         {
            @Override
            public ICPOptimizationParameters getICPOptimizationParameters()
            {
               return new AtlasICPOptimizationParameters(false)
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
   public void testStepInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testStepInPlace();
   }

   @Tag("humanoid-flat-ground")
   @Test
   @Override
   public void testStepInPlaceWithPush() throws SimulationExceededMaximumTimeException
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
