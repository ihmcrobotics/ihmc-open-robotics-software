package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.AlwaysSucessfulAction;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.FallbackNode;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.NonReactiveLoopSequenceNode;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.TimedExpirationCondition;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

public class BehaviorTreeNonReactiveTest
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

      assertEquals(RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("01", output.getValue());
      assertEquals(RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("012", output.getValue());
      assertEquals(RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("0123", output.getValue());

      assertEquals(RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("01231", output.getValue());
      assertEquals(RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("012312", output.getValue());
      assertEquals(RUNNING, nonReactiveLoopSequenceNode.tick());
      assertEquals("0123123", output.getValue());
   }

   @Test
   public void testExpirationCondition()
   {
      TimedExpirationCondition timedExpirationCondition = new TimedExpirationCondition(1.0);
      assertEquals(FAILURE, timedExpirationCondition.tick());
      timedExpirationCondition.renew();
      assertEquals(SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(500);
      assertEquals(SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(520);
      assertEquals(FAILURE, timedExpirationCondition.tick());
      timedExpirationCondition.renew();
      assertEquals(SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(500);
      assertEquals(SUCCESS, timedExpirationCondition.tick());
      ThreadTools.sleep(520);
      assertEquals(FAILURE, timedExpirationCondition.tick());
   }
   @Test
   public void testFallbackNode()
   {
      MutableObject<String> output = new MutableObject<>();
      output.setValue("0");

      BehaviorTreeTestAction testAction1 = new BehaviorTreeTestAction(() -> output.setValue(output.getValue() + "1"));
      BehaviorTreeTestAction testAction2 = new BehaviorTreeTestAction(() -> output.setValue(output.getValue() + "2"));
      BehaviorTreeTestAction testAction3 = new BehaviorTreeTestAction(() -> output.setValue(output.getValue() + "3"));

      FallbackNode fallbackNode = new FallbackNode();
      fallbackNode.addChild(testAction1);
      fallbackNode.addChild(testAction2);
      fallbackNode.addChild(testAction3);

      testAction1.setStatus(SUCCESS);
      testAction2.setStatus(SUCCESS);
      testAction3.setStatus(SUCCESS);

      assertEquals(SUCCESS, fallbackNode.tick());
      assertEquals("01", output.getValue());

      testAction1.setStatus(FAILURE);
      testAction2.setStatus(SUCCESS);
      testAction3.setStatus(SUCCESS);

      assertEquals(SUCCESS, fallbackNode.tick());
      assertEquals("0112", output.getValue());

      testAction1.setStatus(FAILURE);
      testAction2.setStatus(FAILURE);
      testAction3.setStatus(SUCCESS);

      assertEquals(SUCCESS, fallbackNode.tick());
      assertEquals("0112123", output.getValue());

      testAction1.setStatus(FAILURE);
      testAction2.setStatus(FAILURE);
      testAction3.setStatus(FAILURE);

      assertEquals(FAILURE, fallbackNode.tick());
      assertEquals("0112123123", output.getValue());
   }

   public static void main(String[] args)
   {
      new BehaviorTreeNonReactiveTest().testLoopSequenceNode();
   }
}
