package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndFootTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndFootTrajectoryMessageTest extends EndToEndFootTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false)
   {
      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      { // FIXME Hack to disable joint damping so it is easier to perform assertions on tracking. It'd be good if that was available at construction of the sim.
         return createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
      };
   };

   @Tag("controller-api-slow")
   @Test
   @Override
   public void testCustomControlPoint() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlPoint();
   }

   @Tag("controller-api")
   @Test
   @Override
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api")
   @Test
   @Override
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }

   @Tag("controller-api")
   @Test
   @Override
   public void testQueuedMessages() throws SimulationExceededMaximumTimeException
   {
      super.testQueuedMessages();
   }

   @Tag("controller-api-slow")
   @Test
   @Override
   public void testQueueStoppedWithOverrideMessage() throws SimulationExceededMaximumTimeException
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Tag("controller-api-slow")
   @Test
   @Override
   public void testQueueWithWrongPreviousId() throws SimulationExceededMaximumTimeException
   {
      super.testQueueWithWrongPreviousId();
   }

   @Tag("controller-api")
   @Test
   @Override
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }

   @Tag("controller-api")
   @Test
   @Override
   public void testPickUpAndPutDown() throws SimulationExceededMaximumTimeException
   {
      super.testPickUpAndPutDown();
   }

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
}
