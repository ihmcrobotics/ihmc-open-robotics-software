package us.ihmc.robotics.taskExecutor;

import java.util.List;

/**
 * Block diagram representation of a PipeLine:
 * <pre>
 *          |
 *        -----
 *        | A |              Stage 1
 *        -----
 *          |            \
 *    -------------       |
 *    |     |     |       |
 *  ----- ------ ------   |
 *  |   | | B0 | | C0 |   |
 *  | A | ------ ------   |
 *  |   |   |      |      |  Stage 2
 *  ----- ------   |      |
 *    |   | B1 | ------   |
 *    |   ------ | C1 |   |
 *    |     |    ------   |
 *    |     |      |      |
 *    --------------      |
 *          |            /
 *    -------------
 *    |           |
 *  -----       -----
 *  | A |       | B |        Stage 3
 *  -----       -----
 *    |           |
 *    -------------
 *          |
 *        -----
 *        | A |              Stage 4
 *        -----
 *          |
 * </pre>
 * 
 * <p>
 *  Each block represents a Task to be executed.
 * </p>
 * <p>
 *  The tasks are submitted to the master TaskExecutor and can be executed sequentially (as stages 1 and 4) or in parallel (as stages 2 and 3).
 * The parallel tasks run using ParallelTask which uses "slave TaskExecutors" (one slave TaskExecutor per task to be parallelized).
 * </p>
 * <p>
 *  Note about Stage 2: As for each key (A, B, and C) a slave TaskExecutor is used, it is possible to execute a pile of sequential tasks associated with each of the key.
 * When all the slave TaskExecutors are done with their pile of tasks, the master TaskExecutor can go to the next stage, for instance Stage 3.
 * </p> 
 * <p>
 *  Note about how to create the transition from Stage 2 to Stage 3: To be able to execute two consecutive stages consisting of parallel tasks, it is necessary to call the methods requestNextStage().
 * </p> 
 * 
 */
public class PipeLine<T>
{
   private static final boolean DEBUG = false;
   /** The master TaskExecutor is used to execute the different stages sequentially. */
   private final TaskExecutor masterTaskExecutor = new TaskExecutor();
   private final NullTask nullTask = new NullTask();

   /**
    * Create a new stage in the master TaskExecutor.
    * Helpful especially when two consecutive stages have parallel tasks.
    */
   public void requestNewStage()
   {
      submitSingleTaskStage(nullTask);
   }

   /**
    * Submit a new single Task stage that will be executed in a new stage.
    */
   public void submitSingleTaskStage(Task singleTaskStage)
   {
      if (DEBUG)
         System.out.println(getClass().getSimpleName() + ": new task submitted. Type: " + singleTaskStage.getClass().getSimpleName());
      masterTaskExecutor.submit(singleTaskStage);
   }

   /**
    * Submit a new Task to be parallelized. Two possible cases:
    * <p>
    * <li>
    *  If the last stage is a single task stage, a new stage for parallel tasking is created and the taskToParallelize is submitted to a slave TaskExecutor.
    * </li>
    * <li>
    *  If the last stage is a parallel task stage, the taskToParallelize is submitted to the corresponding slave TaskExecutor using the executorKey.
    *  To create a new stage, use first {@link #requestNewStage()}.
    * </li>
    * </p>
    * @param executorKey
    * @param taskToParallelize
    */
   @SuppressWarnings("unchecked")
   public void submitTaskForPallelPipesStage(T executorKey, Task taskToParallelize)
   {
      ParallelTask<T> lastParallelTask;
      if (masterTaskExecutor.getLastTask() instanceof ParallelTask)
      {
         // In that case:
         // - the last stage of the master TaskExecutor is a stage of parallel tasks.
         // Thus, insert the new task to be parallelized in the last stage.
         lastParallelTask = (ParallelTask<T>) masterTaskExecutor.getLastTask();
      }
      else
      {
         // In that case:
         // - the last task of the masterTaskExecutor is a single task stage.
         // Thus, create a new stage in the master TaskExecutor for a new series of parallel tasks.

         // Create a new stage of parallel tasks.
         lastParallelTask = new ParallelTask<T>(); //generates garbage, be careful when using in a realtime setting

         // Submit a new stage to the master TaskExecutor
         submitSingleTaskStage(lastParallelTask);
      }

      lastParallelTask.submit(executorKey, taskToParallelize);
   }

   public void doControl()
   {
      masterTaskExecutor.doControl();
   }

   /**
    * Remove all the stages from the master TaskExecutor.
    */
   public void clearAll()
   {
      masterTaskExecutor.clear();
   }

   /**
    * Remove all the stages from the master TaskExecutor except the current stage.
    */
   public void clearAllExceptCurrent()
   {
      masterTaskExecutor.clearAllExceptCurrent();
   }

   
//    DISABLED: deque.get() does not exist except in the 1.5 compatibility version for no good reason
//   /**
//    * Clear all the slave TaskExecutors associated with the given executorKey.
//    * @param executorKey
//    */
//   public void clearAll(T executorKey)
//   {
//      ArrayDeque<Task> stageQueue = masterTaskExecutor.getTaskQueue();
//      
//      
//      for (int i = 0; i < stageQueue.size(); i++)
//      {
//         Task stage = stageQueue.get(i);
//         if (stage instanceof ParallelTask<?>)
//         {
//            @SuppressWarnings("unchecked")
//            ParallelTask<T> stageOfParallelTasks = (ParallelTask<T>) stage;
//            stageOfParallelTasks.clear(executorKey);
//         }
//      }
//   }

   /**
    * Clear the slave TaskExecutor associated with the given executorKey in the current stage.
    * @param executorKey
    */
   public void clear(T executorKey)
   {
      Task currentStage = masterTaskExecutor.getCurrentTask();
      if (currentStage instanceof ParallelTask<?>)
      {
         @SuppressWarnings("unchecked")
         ParallelTask<T> stageOfParallelTasks = (ParallelTask<T>) currentStage;
         stageOfParallelTasks.clear(executorKey);
      }
   }

   public void submitAll(List<Task> tasks)
   {
      for (int i = 0; i < tasks.size(); i++)
      {
         submitSingleTaskStage(tasks.get(i));
      }
   }

   public void submitAll(T executorKey, List<Task> tasks)
   {
      for (int i = 0; i < tasks.size(); i++)
      {
         submitTaskForPallelPipesStage(executorKey, tasks.get(i));
      }
   }

   // TODO To be tested
   public boolean isDone()
   {
      return masterTaskExecutor.isDone();
   }

   public Task getCurrentStage()
   {
      return masterTaskExecutor.getCurrentTask();
   }
}
