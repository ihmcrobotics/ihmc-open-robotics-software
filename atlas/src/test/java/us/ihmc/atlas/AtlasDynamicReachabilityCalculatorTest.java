package us.ihmc.atlas;

import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarDynamicReachabilityCalculatorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.DynamicReachabilityParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasDynamicReachabilityCalculatorTest extends AvatarDynamicReachabilityCalculatorTest
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
                  public boolean editStepTimingForReachability()
                  {
                     return true;
                  }

                  @Override
                  public double nominalHeightAboveAnkle()
                  {
                     return 0.84; // height for straight leg steps
                  }

                  @Override
                  public double maximumHeightAboveAnkle()
                  {
                     return 0.92; // height for straight leg steps
                  }

                  @Override
                  public DynamicReachabilityParameters getDynamicReachabilityParameters()
                  {
                     return new DynamicReachabilityParameters()
                     {
                        @Override
                        public double getMaximumDesiredKneeBend()
                        {
                           return 0.4;
                        }
                     };
                  }
               };
            }

         };

      return atlasRobotModel;
   }

   public static void main(String[] args)
   {
      AtlasDynamicReachabilityCalculatorTest test = new AtlasDynamicReachabilityCalculatorTest();
      try
      {
         test.testForwardWalkingMedium();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }
}
