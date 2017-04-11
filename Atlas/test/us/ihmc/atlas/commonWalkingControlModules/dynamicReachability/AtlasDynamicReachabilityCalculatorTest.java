package us.ihmc.atlas.commonWalkingControlModules.dynamicReachability;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.AvatarDynamicReachabilityCalculatorTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasDynamicReachabilityCalculatorTest extends AvatarDynamicReachabilityCalculatorTest
{
   protected DRCRobotModel getRobotModel()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false)
         {
            @Override
            public WalkingControllerParameters getWalkingControllerParameters()
            {
               return new AtlasWalkingControllerParameters(DRCRobotModel.RobotTarget.SCS, getJointMap(), getContactPointParameters())
               {
                  @Override
                  public boolean editStepTimingForReachability()
                  {
                     return true;
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
         test.testForwardWalking();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }
}
