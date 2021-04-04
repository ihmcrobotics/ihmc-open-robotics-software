package us.ihmc.humanoidBehaviors.tools.behaviorTrees;

import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeAction;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.RUNNING;

public class BehaviorTreeReactiveTestAction implements BehaviorTreeAction
{
   private final int stepDuration;
   private final int taskSteps;
   private final int startIndex;
   private BehaviorTreeNodeStatus resultStatus;
   private final AtomicReference<String> output;
   private final Notification stepNotification;

   private final AtomicReference<BehaviorTreeNodeStatus> currentStatus = new AtomicReference<>();

   public BehaviorTreeReactiveTestAction(int stepDuration,
                                         int taskSteps,
                                         int startIndex,
                                         BehaviorTreeNodeStatus resultStatus,
                                         AtomicReference<String> testOutput,
                                         Notification stepNotification)
   {
      this.stepDuration = stepDuration;
      this.taskSteps = taskSteps;
      this.startIndex = startIndex;
      this.resultStatus = resultStatus;
      this.output = testOutput;
      this.stepNotification = stepNotification;
   }

   @Override
   public synchronized BehaviorTreeNodeStatus tick()
   {
      if (currentStatus.get() == null)
      {
         currentStatus.set(RUNNING);
         ThreadTools.startAThread(this::sleepTask, "sleepTask");
      }

      return currentStatus.get();
   }

   private void sleepTask()
   {
      for (int i = 0; i < taskSteps; i++)
      {
         ThreadTools.sleep(stepDuration);
         output.set(output.get() + (startIndex + i));
         stepNotification.set();
      }

      synchronized (this)
      {
         currentStatus.set(resultStatus);
      }
   }

   public synchronized void reset(BehaviorTreeNodeStatus resultStatus)
   {
      this.resultStatus = resultStatus;

      if (currentStatus.get() != RUNNING) // TODO: make sure this is thread safe
      {
         currentStatus.set(null);
      }
   }
}
