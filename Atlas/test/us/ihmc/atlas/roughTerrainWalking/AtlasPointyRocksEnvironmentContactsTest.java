package us.ihmc.atlas.roughTerrainWalking;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.roughTerrainWalking.HumanoidPointyRocksEnvironmentContactsTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW})
public class AtlasPointyRocksEnvironmentContactsTest extends HumanoidPointyRocksEnvironmentContactsTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);

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

   @Override
   protected DRCRobotModel getRobotModel(int xContactPoints, int yContactPoints, boolean createOnlyEdgePoints)
   {
      FootContactPoints simulationContactPoints = new AdditionalSimulationContactPoints(xContactPoints, yContactPoints, createOnlyEdgePoints, true);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false, simulationContactPoints);
      return robotModel;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 137.5, categoriesOverride = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
   @Test(timeout = 690000)
   public void testWalkingOnLinesInEnvironment() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnLinesInEnvironment();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 69.7, categoriesOverride = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
   @Test(timeout = 350000)
   public void testWalkingOnPointInEnvironment() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnPointInEnvironment();
   }
}
