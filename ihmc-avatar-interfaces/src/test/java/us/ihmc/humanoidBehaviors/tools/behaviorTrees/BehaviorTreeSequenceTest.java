package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.AlwaysSucessfulAction;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.NonReactiveLoopSequenceNode;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.TimedExpirationCondition;

import static org.junit.jupiter.api.Assertions.*;

public class BehaviorTreeSequenceTest
{
   @Test
   public void testLoopSequenceNode()
   {
      MutableObject<String> output = new MutableObject<>();
      output.setValue("0");

      NonReactiveLoopSequenceNode nonReactiveLoopSequenceNode = new NonReactiveLoopSequenceNode();
      nonReactiveLoopSequenceNode.addChild(new AlwaysSucessfulAction(() -> output.setValue(output.getValue() + "1")));
      nonReactiveLoopSequenceNode.addChild(new AlwaysSucessfulAction(() -> output.setValue(output.getValue() + "2")));
      nonReactiveLoopSequenceNode.addChild(new AlwaysSucessfulAction(() -> output.setValue(output.getValue() + "3")));

      assertEquals("0", output.getValue());

      assertEquals(BehaviorTreeNodeStatus.RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("01", output.getValue());
      assertEquals(BehaviorTreeNodeStatus.RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("012", output.getValue());
      assertEquals(BehaviorTreeNodeStatus.RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("0123", output.getValue());

      assertEquals(BehaviorTreeNodeStatus.RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("01231", output.getValue());
      assertEquals(BehaviorTreeNodeStatus.RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("012312", output.getValue());
      assertEquals(BehaviorTreeNodeStatus.RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("0123123", output.getValue());
   }

   @Test
   public void testExpirationCondition()
   {
      TimedExpirationCondition timedExpirationCondition = new TimedExpirationCondition(1.0);
      assertEquals(BehaviorTreeNodeStatus.FAILURE, timedExpirationCondition.tick());
      timedExpirationCondition.renew();
      assertEquals(BehaviorTreeNodeStatus.SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(500);
      assertEquals(BehaviorTreeNodeStatus.SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(520);
      assertEquals(BehaviorTreeNodeStatus.FAILURE, timedExpirationCondition.tick());
      timedExpirationCondition.renew();
      assertEquals(BehaviorTreeNodeStatus.SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(500);
      assertEquals(BehaviorTreeNodeStatus.SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(520);
      assertEquals(BehaviorTreeNodeStatus.FAILURE, timedExpirationCondition.tick());
   }

   public static void main(String[] args)
   {
      new BehaviorTreeSequenceTest().testLoopSequenceNode();
   }
}
