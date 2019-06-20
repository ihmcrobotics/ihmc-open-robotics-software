package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseDoNothingTest;
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

   @Test
   public void testDoNothingGroundContactPoints() throws SimulationExceededMaximumTimeException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      super.testDoNothing1();
   }

   @Test
   public void testDoNothingShapeCollision() throws SimulationExceededMaximumTimeException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false, true);
      super.testDoNothing1();
   }
}
