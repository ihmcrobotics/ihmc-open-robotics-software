package us.ihmc.atlas.controllerAPI;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
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
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 200000)
   public void testSixDoFMovementsOfPelvis() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testSixDoFMovementsOfPelvis();
   }

   @ContinuousIntegrationTest(estimatedDuration = 91.2, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 460000)
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }
}
