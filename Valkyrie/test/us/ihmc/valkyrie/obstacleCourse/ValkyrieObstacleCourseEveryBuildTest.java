package us.ihmc.valkyrie.obstacleCourse;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseEveryBuildTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class ValkyrieObstacleCourseEveryBuildTest extends DRCObstacleCourseEveryBuildTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 72.0)
   @Test(timeout = 360000)
   public void testSimpleFlatGroundScript() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScript();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 53.4)
   @Test(timeout = 270000)
   public void testWalkingUpToRampWithLongSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithLongSteps();
   }
}
