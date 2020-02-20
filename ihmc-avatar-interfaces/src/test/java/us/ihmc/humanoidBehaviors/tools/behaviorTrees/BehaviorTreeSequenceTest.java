package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.AlwaysSucessfulAction;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.NonReactiveLoopSequenceNode;

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

   public static void main(String[] args)
   {
      new BehaviorTreeSequenceTest().testLoopSequenceNode();
   }
}
