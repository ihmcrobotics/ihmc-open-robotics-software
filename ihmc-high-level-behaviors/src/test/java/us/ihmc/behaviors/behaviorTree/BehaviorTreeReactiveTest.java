package us.ihmc.behaviors.behaviorTree;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

public class BehaviorTreeReactiveTest
{
   @Test
   public void testExpirationCondition()
   {
      TimedExpirationCondition timedExpirationCondition = new TimedExpirationCondition(1.0);
      assertEquals(FAILURE, timedExpirationCondition.tickAndGetStatus());
      timedExpirationCondition.update();
      assertEquals(SUCCESS, timedExpirationCondition.tickAndGetStatus());
      ThreadTools.sleep(500);
      assertEquals(SUCCESS, timedExpirationCondition.tickAndGetStatus());
      ThreadTools.sleep(520);
      assertEquals(FAILURE, timedExpirationCondition.tickAndGetStatus());
      timedExpirationCondition.update();
      assertEquals(SUCCESS, timedExpirationCondition.tickAndGetStatus());
      ThreadTools.sleep(500);
      assertEquals(SUCCESS, timedExpirationCondition.tickAndGetStatus());
      ThreadTools.sleep(520);
      assertEquals(FAILURE, timedExpirationCondition.tickAndGetStatus());
   }

   @Test
   public void testReactiveSequenceNode()
   {
//      loadClassesHack(); // enable this if the test is flaky

      AtomicReference<String> output = new AtomicReference<>();
      output.set("0");

      Notification stepNotification = new Notification();

      int taskStepDuration = 100;

      BehaviorTreeReactiveTestAction testAction1 = new BehaviorTreeReactiveTestAction(taskStepDuration, 3, 10, SUCCESS, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction2 = new BehaviorTreeReactiveTestAction(taskStepDuration, 4, 20, SUCCESS, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction3 = new BehaviorTreeReactiveTestAction(taskStepDuration, 2, 30, SUCCESS, output, stepNotification);

      SequenceNode sequenceNode = new SequenceNode();
      sequenceNode.getChildren().add(testAction1);
      sequenceNode.getChildren().add(testAction2);
      sequenceNode.getChildren().add(testAction3);

      printAndAssert(output, "0");

      // use blocking polls to prevent flakiness

      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222330");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122233031");
      assertEquals(SUCCESS, sequenceNode.tickAndGetStatus());
      printAndAssert(output, "0101112202122233031");
   }

   @Test
   public void testReactiveSequenceNode2()
   {
      //      loadClassesHack(); // enable this if the test is flaky

      AtomicReference<String> output = new AtomicReference<>();
      output.set("0");

      Notification stepNotification = new Notification();

      int taskStepDuration = 100;

      BehaviorTreeReactiveTestAction testAction1 = new BehaviorTreeReactiveTestAction(taskStepDuration, 3, 10, SUCCESS, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction2 = new BehaviorTreeReactiveTestAction(taskStepDuration, 4, 20, SUCCESS, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction3 = new BehaviorTreeReactiveTestAction(taskStepDuration, 2, 30, SUCCESS, output, stepNotification);

      SequenceNode sequenceNode = new SequenceNode();
      sequenceNode.getChildren().add(testAction1);
      sequenceNode.getChildren().add(testAction2);
      sequenceNode.getChildren().add(testAction3);

      printAndAssert(output, "0");

      // use blocking polls to prevent flakiness

      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112");

      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223");

      testAction2.reset(FAILURE);

      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222320");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122232021");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223202122");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222320212223");

      assertEquals(FAILURE, sequenceNode.tickAndGetStatus());
      testAction2.reset(SUCCESS);

      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122232021222320");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223202122232021");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222320212223202122");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122232021222320212223");

      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223202122232021222330");
      assertEquals(RUNNING, sequenceNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222320212223202122233031");
      assertEquals(SUCCESS, sequenceNode.tickAndGetStatus());
      printAndAssert(output, "01011122021222320212223202122233031");
   }

   @Test
   public void testReactiveFallbackNode()
   {
       // start threads in actions

      AtomicReference<String> output = new AtomicReference<>();
      output.set("0");

      Notification stepNotification = new Notification();

      int taskStepDuration = 100;

      BehaviorTreeReactiveTestAction testAction1 = new BehaviorTreeReactiveTestAction(taskStepDuration, 3, 10, FAILURE, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction2 = new BehaviorTreeReactiveTestAction(taskStepDuration, 4, 20, FAILURE, output, stepNotification);
      BehaviorTreeReactiveTestAction testAction3 = new BehaviorTreeReactiveTestAction(taskStepDuration, 2, 30, FAILURE, output, stepNotification);

      FallbackNode fallbackNode = new FallbackNode();
      fallbackNode.getChildren().add(testAction1);
      fallbackNode.getChildren().add(testAction2);
      fallbackNode.getChildren().add(testAction3);

      printAndAssert(output, "0");

      // use blocking polls to prevent flakiness

      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "010111220212223");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "01011122021222330");
      assertEquals(RUNNING, fallbackNode.tickAndGetStatus());
      stepNotification.blockingPoll();
      printAndAssert(output, "0101112202122233031");
      assertEquals(FAILURE, fallbackNode.tickAndGetStatus());
      printAndAssert(output, "0101112202122233031");
   }

   private void loadClassesHack()
   {
      // load classes ahead of time
      {
         AtomicReference<String> dummy = new AtomicReference<>();
         dummy.set("asd");
         new BehaviorTreeReactiveTestAction(10, 5, 10, SUCCESS, dummy, new Notification()).tick();
      }
   }

   private void printAndAssert(AtomicReference<String> output, String s)
   {
      String outputLocal = output.get();
      LogTools.info(outputLocal);
      assertEquals(s, outputLocal);
   }
}
