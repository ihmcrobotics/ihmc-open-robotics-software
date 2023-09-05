package us.ihmc.behaviors.lookAndStep;

import behavior_msgs.msg.dds.StatusLogMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Test;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.ros2.ROS2Topic;

import static org.junit.jupiter.api.Assertions.*;

public class BehaviorTaskSuppressorTest
{
   @Test
   public void testBasicSuppression()
   {
      MutableInt num1 = new MutableInt();
      MutableInt num2 = new MutableInt();
      MutableInt num3 = new MutableInt();

      StatusLogger statusLogger = new StatusLogger(this::publishToUI);

      BehaviorTaskSuppressor suppressor = new BehaviorTaskSuppressor(statusLogger, "Test task");

      suppressor.addCondition("Num 1 <= 0", () -> num1.getValue() <= 0, num1::increment);

      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");

      suppressor.addCondition("Num 2 <= 1", () -> num2.getValue() <= 1, num2::increment);

      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");

      suppressor.addCondition("Num 3 != 3", () -> num3.getValue() != 3, num3::increment);

      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertTrue(suppressor.evaulateShouldAccept(), "Evaluation failure");
      num3.increment();
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
      assertFalse(suppressor.evaulateShouldAccept(), "Evaluation failure");
   }

   private void publishToUI(ROS2Topic<StatusLogMessage> statusLogMessageROS2Topic, StatusLogMessage statusLogMessage)
   {

   }
}
