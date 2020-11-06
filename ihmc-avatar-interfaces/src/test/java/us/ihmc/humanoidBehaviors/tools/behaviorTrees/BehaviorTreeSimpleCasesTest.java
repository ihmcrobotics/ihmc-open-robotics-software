package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;

import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeAction;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.SequenceNode;

class BehaviorTreeSimpleCasesTest
{

   @Test
   void testSimpleCase()
   {
      MutableObject<String> output = new MutableObject<>("");
      SequenceNode tree = new SequenceNode();

      BehaviorTreeAction findBall = new BehaviorTreeAction()
      {
         private int attempt = 0;
         private int numberOfAttempts = 3;
         
         @Override
         public BehaviorTreeNodeStatus tick()
         {
            output.setValue(output.getValue() + "F");
            
            if (attempt >= numberOfAttempts)
               return BehaviorTreeNodeStatus.SUCCESS;
            attempt++;
            return BehaviorTreeNodeStatus.RUNNING;
         }
      };

      BehaviorTreeAction pickBall = new BehaviorTreeAction()
      {
         @Override
         public BehaviorTreeNodeStatus tick()
         {
            output.setValue(output.getValue() + "P");

            return BehaviorTreeNodeStatus.SUCCESS;
         }
      };

      BehaviorTreeAction dropBall = new BehaviorTreeAction()
      {
         @Override
         public BehaviorTreeNodeStatus tick()
         {
            output.setValue(output.getValue() + "D");

            return BehaviorTreeNodeStatus.SUCCESS;
         }
      };
      tree.addChild(findBall);
      tree.addChild(pickBall);
      tree.addChild(dropBall);

      BehaviorTreeNodeStatus status = tree.tick();
      assertEquals(status, BehaviorTreeNodeStatus.RUNNING);
      assertEquals(output.getValue(), "F");
      
      status = tree.tick();
      assertEquals(status, BehaviorTreeNodeStatus.RUNNING);
      assertEquals(output.getValue(), "FF");
      
      status = tree.tick();
      assertEquals(status, BehaviorTreeNodeStatus.RUNNING);
      assertEquals(output.getValue(), "FFF");
      
      status = tree.tick();
      assertEquals(status, BehaviorTreeNodeStatus.SUCCESS);
      assertEquals(output.getValue(), "FFFFPD");
      
      status = tree.tick();
      assertEquals(status, BehaviorTreeNodeStatus.SUCCESS);
      assertEquals(output.getValue(), "FFFFPDFPD");
   }
   


}
