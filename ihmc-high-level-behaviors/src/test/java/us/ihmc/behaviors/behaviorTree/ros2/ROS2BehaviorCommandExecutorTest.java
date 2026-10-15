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
   public void testMimicStartsOnDrainNotOnBehaviorUpdate()
   {
      List<BehaviorResultMessage> results = new ArrayList<>();
      ROS2BehaviorCommandExecutor executor = new ROS2BehaviorCommandExecutor(results::add);
      AtomicInteger starts = new AtomicInteger();
      executor.setMimicStarter((requestId, name, closestPerson) ->
      {
         starts.incrementAndGet();
         return new StartResult(BehaviorResultMessage.ACCEPTED, name, false);
      });

      BehaviorCommandMessage command = new BehaviorCommandMessage();
      command.setRequestId(7);
      command.setKind(BehaviorCommandMessage.MIMIC);
      command.setName("wave");
      executor.enqueue(command);

      executor.update();
      assertEquals(0, starts.get());
      assertTrue(results.isEmpty());

      executor.drain();
      assertEquals(1, starts.get());
      assertEquals(1, results.size());
      assertEquals(7, results.get(0).getRequestId());
      assertEquals(BehaviorResultMessage.ACCEPTED, results.get(0).getPhase());
      assertEquals("wave", results.get(0).getSummaryAsString());
   }
}
