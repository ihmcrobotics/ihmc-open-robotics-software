package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.SequenceNode;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.TimedExpirationCondition;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

public class BehaviorTreeReactiveTest
{
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
   public void testReactiveSequenceNode()
   {
      // load classes ahead of time
      {
         AtomicReference<String> dummy = new AtomicReference<>();
         dummy.set("asd");
         new BehaviorTreeReactiveTestAction(10, 5, 10, SUCCESS, dummy, new Notification()).tick();
      }

      AtomicReference<String> output = new AtomicReference<>();
      output.set("0");

      Notification stepNotification = new Notification();

      int taskStepDuration = 100;

      BehaviorTreeReactiveTestAction testAction1 = new BehaviorTreeReactiveTestAction(taskStepDuration, 3, 10, SUCCESS, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction2 = new BehaviorTreeReactiveTestAction(taskStepDuration, 4, 20, SUCCESS, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction3 = new BehaviorTreeReactiveTestAction(taskStepDuration, 2, 30, SUCCESS, output, stepNotification);

      SequenceNode sequenceNode = new SequenceNode();
      sequenceNode.addChild(testAction1);
      sequenceNode.addChild(testAction2);
      sequenceNode.addChild(testAction3);

      printAndAssert(output, "0");

//      assertEquals(RUNNING, sequenceNode.tick()); // get it started
//      ThreadTools.sleep(40); // go halfway between task step duration to avoid race conditions

//      while (true)
//      {
//         Stopwatch stopwatch = new Stopwatch().start();
//         BehaviorTreeNodeStatus status = sequenceNode.tick();
//         LogTools.info("Tick took {}", stopwatch.lap());
//         stepNotification.blockingPoll();
//         LogTools.info("Blocking poll took {}", stopwatch.lap());
//         LogTools.info("{}: {}", status.name(), output.get());
////         ThreadTools.sleep(taskStepDuration);
////         System.out.println(status.name() + ": " + output.get());
//      }

      // use blocking polls to prevent flakiness

      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "010");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222330");
      assertEquals(RUNNING, sequenceNode.tick());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122233031");
      assertEquals(SUCCESS, sequenceNode.tick());
      printAndAssert(output, "0101112202122233031");
   }

   private void printAndAssert(AtomicReference<String> output, String s)
   {
      String outputLocal = output.get();
      LogTools.info(outputLocal);
      assertEquals(s, outputLocal);
   }

   @Test
   public void testReactiveFallbackNode()
   {
       // start threads in actions

   }
}
