package us.ihmc.humanoidBehaviors.lookAndStep;

public class LookAndStepFootstepPlanningModule implements Builder
{

   private LookAndStepFootstepPlanningTask task;

   public LookAndStepFootstepPlanningModule()
   {
      SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

      task = new LookAndStepFootstepPlanningTask();
   }

   public void accept()
   {

   }

   private void evaluateAndRun()
   {
      validate();

      // setters

      task.run();
   }
}
