package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.BehaviorCommandMessage;
import behavior_msgs.BehaviorResultMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorCommandExecutor.StartResult;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ROS2BehaviorCommandExecutorTest
{
   @Test
   public void testMimicStartsOnMimicUpdate()
   {
      List<BehaviorResultMessage> results = new ArrayList<>();
      ROS2BehaviorCommandExecutor executor = new ROS2BehaviorCommandExecutor(results::add);
      AtomicInteger starts = new AtomicInteger();
      executor.setStarter(BehaviorCommandMessage.MIMIC, (requestId, name, closestPerson) ->
      {
         starts.incrementAndGet();
         return new StartResult(BehaviorResultMessage.ACCEPTED, name, false);
      });

      BehaviorCommandMessage command = new BehaviorCommandMessage();
      command.setRequestId(7);
      command.setKind(BehaviorCommandMessage.MIMIC);
      command.setName("wave");
      executor.enqueue(command);

      executor.update(BehaviorCommandMessage.TREE);
      assertEquals(0, starts.get());
      assertTrue(results.isEmpty());

      executor.update(BehaviorCommandMessage.FOLLOW);
      assertEquals(0, starts.get());
      assertTrue(results.isEmpty());

      executor.update(BehaviorCommandMessage.MIMIC);
      assertEquals(1, starts.get());
      assertEquals(1, results.size());
      assertEquals(7, results.get(0).getRequestId());
      assertEquals(BehaviorResultMessage.ACCEPTED, results.get(0).getPhase());
      assertEquals("wave", results.get(0).getSummaryAsString());
   }

   @Test
   public void testMimicBehindATreeStillStarts()
   {
      List<BehaviorResultMessage> results = new ArrayList<>();
      ROS2BehaviorCommandExecutor executor = new ROS2BehaviorCommandExecutor(results::add);
      AtomicInteger starts = new AtomicInteger();
      executor.setStarter(BehaviorCommandMessage.MIMIC, (requestId, name, closestPerson) ->
      {
         starts.incrementAndGet();
         return new StartResult(BehaviorResultMessage.ACCEPTED, name, false);
      });

      BehaviorCommandMessage tree = new BehaviorCommandMessage();
      tree.setRequestId(1);
      tree.setKind(BehaviorCommandMessage.TREE);
      tree.setName("skill/GoHome.json");
      executor.enqueue(tree);

      BehaviorCommandMessage mimic = new BehaviorCommandMessage();
      mimic.setRequestId(2);
      mimic.setKind(BehaviorCommandMessage.MIMIC);
      mimic.setName("wave");
      executor.enqueue(mimic);

      executor.update(BehaviorCommandMessage.MIMIC);
      assertEquals(1, starts.get());
      assertEquals(1, results.size());
      assertEquals(BehaviorResultMessage.ACCEPTED, results.get(0).getPhase());
      assertEquals("wave", results.get(0).getSummaryAsString());
   }

   @Test
   public void testFollowStartsOnFollowUpdate()
   {
      List<BehaviorResultMessage> results = new ArrayList<>();
      ROS2BehaviorCommandExecutor executor = new ROS2BehaviorCommandExecutor(results::add);
      AtomicInteger starts = new AtomicInteger();
      executor.setStarter(BehaviorCommandMessage.FOLLOW, (requestId, name, closestPerson) ->
      {
         starts.incrementAndGet();
         return new StartResult(BehaviorResultMessage.ACCEPTED, name, false);
      });

      BehaviorCommandMessage command = new BehaviorCommandMessage();
      command.setRequestId(3);
      command.setKind(BehaviorCommandMessage.FOLLOW);
      command.setName("person");
      executor.enqueue(command);

      executor.update(BehaviorCommandMessage.MIMIC);
      assertEquals(0, starts.get());
      assertTrue(results.isEmpty());

      executor.update(BehaviorCommandMessage.FOLLOW);
      assertEquals(1, starts.get());
      assertEquals(BehaviorResultMessage.ACCEPTED, results.get(0).getPhase());
      assertEquals("person", results.get(0).getSummaryAsString());
   }

   @Test
   public void testFollowWithoutStarterIsRejectedOnTreeUpdate()
   {
      List<BehaviorResultMessage> results = new ArrayList<>();
      ROS2BehaviorCommandExecutor executor = new ROS2BehaviorCommandExecutor(results::add);

      BehaviorCommandMessage command = new BehaviorCommandMessage();
      command.setRequestId(4);
      command.setKind(BehaviorCommandMessage.FOLLOW);
      command.setName("person");
      executor.enqueue(command);

      executor.update(BehaviorCommandMessage.TREE);
      assertEquals(1, results.size());
      assertEquals(4, results.get(0).getRequestId());
      assertEquals(BehaviorResultMessage.REJECTED, results.get(0).getPhase());
      assertTrue(results.get(0).getRejected());
      assertEquals("Follow is not running", results.get(0).getSummaryAsString());
   }
}
