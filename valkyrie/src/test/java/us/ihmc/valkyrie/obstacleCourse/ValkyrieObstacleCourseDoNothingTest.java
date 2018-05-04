package us.ihmc.valkyrie.obstacleCourse;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseDoNothingTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieObstacleCourseDoNothingTest extends DRCObstacleCourseDoNothingTest
{
   private ValkyrieRobotModel robotModel;

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

   @ContinuousIntegrationTest(estimatedDuration = 20.3)
   @Test(timeout = 100000)
   public void testDoNothingGroundContactPoints() throws SimulationExceededMaximumTimeException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      super.testDoNothing1();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.3)
   @Test(timeout = 100000)
   public void testDoNothingShapeCollision() throws SimulationExceededMaximumTimeException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false, true);
      super.testDoNothing1();
   }
}
