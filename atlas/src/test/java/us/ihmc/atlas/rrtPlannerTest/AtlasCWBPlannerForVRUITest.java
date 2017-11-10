package us.ihmc.atlas.rrtPlannerTest;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.wholeBodyPlanningTest.AvatarCWBPlannerForVRUITest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasCWBPlannerForVRUITest extends AvatarCWBPlannerForVRUITest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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
   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test(timeout = 83000)
   public void testForReachabilityEETraj() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForReachabilityEETraj();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 63000)
   public void testForWayPointsTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForWayPointsTrajectory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testForReachability() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForReachability();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 200.0)
   @Test(timeout = 230000)
   public void testForBehavior() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForBehavior();
   }
}
