package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

public class BehaviorTreeNonReactiveTest
{
   @Disabled("This functionality is kind of undefined right now. It's not clear what a loop sequence should do or if it should exist.")
   @Test
   public void testLoopSequenceNode()
   {
      MutableObject<String> output = new MutableObject<>();
      output.setValue("0");

      LoopSequenceNode loopSequenceNode = new LoopSequenceNode();
      loopSequenceNode.getChildren().add(new AlwaysSuccessfulAction(() -> output.setValue(output.getValue() + "1")));
      loopSequenceNode.getChildren().add(new AlwaysSuccessfulAction(() -> output.setValue(output.getValue() + "2")));
      loopSequenceNode.getChildren().add(new AlwaysSuccessfulAction(() -> output.setValue(output.getValue() + "3")));

      assertEquals("0", output.getValue());

      assertEquals(RUNNING, loopSequenceNode.tickAndGetStatus());
      assertEquals("01", output.getValue());
      assertEquals(RUNNING, loopSequenceNode.tickAndGetStatus());
      assertEquals("012", output.getValue());
      assertEquals(RUNNING, loopSequenceNode.tickAndGetStatus());
      assertEquals("0123", output.getValue());

      assertEquals(RUNNING, loopSequenceNode.tickAndGetStatus());
      assertEquals("01231", output.getValue());
      assertEquals(RUNNING, loopSequenceNode.tickAndGetStatus());
      assertEquals("012312", output.getValue());
      assertEquals(RUNNING, loopSequenceNode.tickAndGetStatus());
      assertEquals("0123123", output.getValue());
   }

   @Test
   public void testFallbackNode()
   {
      MutableObject<String> output = new MutableObject<>();
      output.setValue("0");

      BehaviorTreeConstantInstantTestAction testAction1 = new BehaviorTreeConstantInstantTestAction(() -> output.setValue(output.getValue() + "1"));
      BehaviorTreeConstantInstantTestAction testAction2 = new BehaviorTreeConstantInstantTestAction(() -> output.setValue(output.getValue() + "2"));
      BehaviorTreeConstantInstantTestAction testAction3 = new BehaviorTreeConstantInstantTestAction(() -> output.setValue(output.getValue() + "3"));

      FallbackNode fallbackNode = new FallbackNode();
      fallbackNode.getChildren().add(testAction1);
      fallbackNode.getChildren().add(testAction2);
      fallbackNode.getChildren().add(testAction3);

      testAction1.setStatus(SUCCESS);
      testAction2.setStatus(SUCCESS);
      testAction3.setStatus(SUCCESS);

      assertEquals(SUCCESS, fallbackNode.tickAndGetStatus());
      assertEquals("01", output.getValue());

      testAction1.setStatus(FAILURE);
      testAction2.setStatus(SUCCESS);
      testAction3.setStatus(SUCCESS);

      assertEquals(SUCCESS, fallbackNode.tickAndGetStatus());
      assertEquals("0112", output.getValue());

      testAction1.setStatus(FAILURE);
      testAction2.setStatus(FAILURE);
      testAction3.setStatus(SUCCESS);

      assertEquals(SUCCESS, fallbackNode.tickAndGetStatus());
      assertEquals("0112123", output.getValue());

      testAction1.setStatus(FAILURE);
      testAction2.setStatus(FAILURE);
      testAction3.setStatus(FAILURE);

      assertEquals(FAILURE, fallbackNode.tickAndGetStatus());
      assertEquals("0112123123", output.getValue());
   }

   public static void main(String[] args)
   {
      new BehaviorTreeNonReactiveTest().testLoopSequenceNode();
   }
}
