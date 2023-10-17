package us.ihmc.behaviors.behaviorTree;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;

class BehaviorTreeSimpleCasesTest
{

   @Test
   void testSimpleCase()
   {
      MutableObject<String> output = new MutableObject<>("");
      SequenceNode tree = new SequenceNode();

      BehaviorTreeNodeExecutor findBall = new LocalOnlyBehaviorTreeNodeExecutor()
      {
         private int attempt = 0;
         private int numberOfAttempts = 3;
         
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            output.setValue(output.getValue() + "F");
            
            if (attempt >= numberOfAttempts)
               return BehaviorTreeNodeStatus.SUCCESS;
            attempt++;
            return BehaviorTreeNodeStatus.RUNNING;
         }
      };

      BehaviorTreeNodeExecutor pickBall = new LocalOnlyBehaviorTreeNodeExecutor()
      {
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            output.setValue(output.getValue() + "P");

            return BehaviorTreeNodeStatus.SUCCESS;
         }
      };

      BehaviorTreeNodeExecutor dropBall = new LocalOnlyBehaviorTreeNodeExecutor()
      {
         @Override
         public BehaviorTreeNodeStatus determineStatus()
         {
            output.setValue(output.getValue() + "D");

            return BehaviorTreeNodeStatus.SUCCESS;
         }
      };
      tree.getChildren().add(findBall);
      tree.getChildren().add(pickBall);
      tree.getChildren().add(dropBall);

      BehaviorTreeNodeStatus status = tree.tickAndGetStatus();
      assertEquals(status, BehaviorTreeNodeStatus.RUNNING);
      assertEquals(output.getValue(), "F");
      
      status = tree.tickAndGetStatus();
      assertEquals(status, BehaviorTreeNodeStatus.RUNNING);
      assertEquals(output.getValue(), "FF");
      
      status = tree.tickAndGetStatus();
      assertEquals(status, BehaviorTreeNodeStatus.RUNNING);
      assertEquals(output.getValue(), "FFF");
      
      status = tree.tickAndGetStatus();
      assertEquals(status, BehaviorTreeNodeStatus.SUCCESS);
      assertEquals(output.getValue(), "FFFFPD");
      
      status = tree.tickAndGetStatus();
      assertEquals(status, BehaviorTreeNodeStatus.SUCCESS);
      assertEquals(output.getValue(), "FFFFPDFPD");
   }
   


}
