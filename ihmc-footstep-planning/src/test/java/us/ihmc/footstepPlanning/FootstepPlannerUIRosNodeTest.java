package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUIRosNode;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.concurrent.atomic.AtomicReference;

import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertTrue;

public class FootstepPlannerUIRosNodeTest
{
   private static final String robotName = "testBot";

   private RealtimeRos2Node localNode;
   private FootstepPlannerUIRosNode uiNode;

   private final AtomicReference<FootstepPlanningRequestPacket> planningRequestReference = new AtomicReference<>(null);

   @Before
   public void setup()
   {
      localNode = ROS2Tools.createRealtimeRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ihmc_footstep_planner_test");
      uiNode = new FootstepPlannerUIRosNode(robotName);
   }

   @After
   public void tearDown() throws Exception
   {
      localNode.destroy();
      uiNode.destroy();

      uiNode = null;
      localNode = null;

      planningRequestReference.set(null);
   }

   @Test
   public void testRequestFootstepPlan() throws Exception
   {
      ROS2Tools.createCallbackSubscription(localNode, FootstepPlanningRequestPacket.class,
                                           ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT),
                                           s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      localNode.spin();

      JavaFXMessager messager = uiNode.getUI().getMessager();

      double timeout = 15.0;
      Point3D startPosition = new Point3D(0.5, 0.5, 0.0);
      Point3D goalPosition = new Point3D(1.0, 1.0, 0.0);
      FootstepPlannerType planningType = FootstepPlannerType.A_STAR;
      // TODO planar regions
      // TODO orientations
      // TODO robot side
      // TODO planning ID

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, goalPosition);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, startPosition);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic, planningType);
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic, timeout);
      // TODO set planar regions
      // TODO set orientations
      // TODO set robot sideyeah
      // TODO set planning ID

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.ComputePathTopic, true);

      int ticks = 0;
      while (planningRequestReference.get() == null)
      {
         ticks++;
         if (ticks > 100)
            assertTrue("Timed out waiting for packet.", false);

         ThreadTools.sleep(10);
      }

      FootstepPlanningRequestPacket packet = planningRequestReference.getAndSet(null);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Start goal positions aren't equal.", startPosition, packet.getStanceFootPositionInWorld(), 1e-5);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("End goal positions aren't equal.", goalPosition, packet.getGoalPositionInWorld(), 1e-5);
      assertEquals("Timeouts aren't equal.", timeout, packet.getTimeout(), 1e-5);
      assertEquals("Planner types aren't equal.", planningType, FootstepPlannerType.fromByte(packet.getRequestedFootstepPlannerType()));
      // TODO test planar regions
      // TODO test orientations
      // TODO test robot side
      // TODO test planning ID
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      planningRequestReference.set(packet);
   }

   @Test
   public void testReceivedGoal() throws Exception
   {
      IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher = ROS2Tools
            .createPublisher(localNode, FootstepPlanningRequestPacket.class,
                             ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));
      localNode.spin();

      JavaFXMessager messager = uiNode.getUI().getMessager();

      AtomicReference<Point3D> goalPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.GoalPositionTopic);
      AtomicReference<Point3D> startPositionReference = messager.createInput(FootstepPlannerUserInterfaceAPI.StartPositionTopic);

      AtomicReference<FootstepPlannerType> planningTypeReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerTypeTopic);
      AtomicReference<Double> timeoutReference = messager.createInput(FootstepPlannerUserInterfaceAPI.PlannerTimeoutTopic);

      double timeout = 15.0;
      Point3D startPosition = new Point3D(0.5, 0.5, 0.0);
      Point3D goalPosition = new Point3D(1.0, 1.0, 0.0);
      FootstepPlannerType planningType = FootstepPlannerType.A_STAR;
      // TODO planar regions
      // TODO timeout
      // TODO orientations
      // TODO robot side
      // TODO planning ID

      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.getStanceFootPositionInWorld().set(startPosition);
      packet.getGoalPositionInWorld().set(goalPosition);
      packet.setRequestedFootstepPlannerType(planningType.toByte());
      packet.setTimeout(15.0);

      // TODO set start position
      // TODO set goal position
      // TODO set planar regions
      // TODO set timeout
      // TODO set orientations
      // TODO set robot side
      // TODO set planning ID

      footstepPlanningRequestPublisher.publish(packet);

      for (int i = 0; i < 10; i++)
      {
         ThreadTools.sleep(10);
      }

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Start goal positions aren't equal.", startPosition, startPositionReference.get(), 1e-5);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("End goal positions aren't equal.", goalPosition, goalPositionReference.get(), 1e-5);
      assertEquals("Timeouts aren't equal.", timeout, timeoutReference.get(), 1e-5);
      assertTrue("Planner types aren't equal.", planningType.equals(planningTypeReference.get()));
      // TODO test planar regions
      // TODO test timeout
      // TODO test orientations
      // TODO test robot side
      // TODO test planning ID

   }
}
