package us.ihmc.humanoidBehaviors.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.humanoidBehaviors.lookAndStep.SingleThreadSizeOneQueueExecutor;

import static org.junit.jupiter.api.Assertions.*;

public class SingleThreadSizeOneQueueExecutorTest
{
   @Test
   public void testOneQueued()
   {
      StringBuilder output = new StringBuilder();

      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor("Test");

      TypedNotification<String> resultOne = new TypedNotification<>();
      TypedNotification<String> resultTwo = new TypedNotification<>();
      Runnable runnableOne = () ->
      {
         output.append("a");
         ThreadTools.sleep(10);
         resultOne.set(output.toString());
      };
      Runnable runnableTwo = () ->
      {
         output.append("b");
         ThreadTools.sleep(10);
         resultTwo.set(output.toString());
      };

      executor.queueExecution(runnableOne);
      executor.queueExecution(runnableTwo);

      resultOne.blockingPoll();
      assertEquals("a", resultOne.read());
      resultTwo.blockingPoll();
      assertEquals("ab", resultTwo.read());
   }

   @Test
   public void testSkipItemInQueue()
   {
      StringBuilder output = new StringBuilder();

      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor("Test");

      TypedNotification<String> resultOne = new TypedNotification<>();
      TypedNotification<String> resultTwo = new TypedNotification<>();
      TypedNotification<String> resultThree = new TypedNotification<>();
      Runnable runnableOne = () ->
      {
         output.append("a");
         ThreadTools.sleep(10);
         resultOne.set(output.toString());
      };
      Runnable runnableTwo = () ->
      {
         output.append("b");
         ThreadTools.sleep(10);
         resultTwo.set(output.toString());
      };
      Runnable runnableThree = () ->
      {
         output.append("c");
         ThreadTools.sleep(10);
         resultThree.set(output.toString());
      };

      executor.queueExecution(runnableOne);
      executor.queueExecution(runnableTwo); // this one should get skipped
      executor.queueExecution(runnableThree);

      resultOne.blockingPoll();
      assertEquals("a", resultOne.read());
      resultThree.blockingPoll();
      assertEquals("ac", resultThree.read());
   }
}
