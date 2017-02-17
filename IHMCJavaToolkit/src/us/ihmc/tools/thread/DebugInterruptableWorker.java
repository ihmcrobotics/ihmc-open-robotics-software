package us.ihmc.tools.thread;

import java.util.Random;

public class DebugInterruptableWorker extends InterruptableWorker
{
   private int result;
   private int endNumber;
   private int countBy;

   public void reset(int startNumber, int endNumber, int countBy)
   {
      super.reset();
      this.endNumber = endNumber;
      this.countBy = countBy;
      result = startNumber;
   }

   @Override
   public STATE doWork()
   {
      while (!isInterrupted())
      {
         result = result + countBy;
         if (result == endNumber)
            return STATE.SUCCESSFULLY_COMPLETED;
         if (result > endNumber)
            return STATE.UNSUCCESSFULLY_COMPLETED;
         busyWork();

         result = result + countBy;
         if (result == endNumber)
            return STATE.SUCCESSFULLY_COMPLETED;
         if (result > endNumber)
            return STATE.UNSUCCESSFULLY_COMPLETED;
         busyWork();

         result = result + countBy;
         if (result == endNumber)
            return STATE.SUCCESSFULLY_COMPLETED;
         if (result > endNumber)
            return STATE.UNSUCCESSFULLY_COMPLETED;
         busyWork();
      }

      return STATE.INTERRUPTED;
   }

   private void busyWork()
   {
      Random random = new Random(result);
      int n = random.nextInt(10000000);
      for (int i = 0; i < 5000000 + n; i++)
      {
         @SuppressWarnings("unused")
         int x = n * n;
      }
   }

   @Override
   public Object getCurrentResult()
   {
      return result;
   }

   public static void main(String[] args)
   {
      // successful completion
      System.out.println("----------------------------------------------");
      System.out.println("creating worker");
      DebugInterruptableWorker worker = new DebugInterruptableWorker();

      System.out.println("resetting worker");
      worker.reset(0, 100, 1);

      System.out.println("starting worker");
      worker.startWorkOnANewThread();

      System.out.println("letting worker finish");
      worker.waitForResult(100000);

      System.out.println("result");
      System.out.println("Worker finished with a state = " + worker.getCurrentState());
      System.out.println("Worker final result = " + worker.getCurrentResult());

      // unsuccessful completion
      System.out.println("----------------------------------------------");
      System.out.println("resetting worker to a non-solution");
      worker.reset(1, 200, 2);

      System.out.println("starting worker");
      worker.startWorkOnANewThread();

      System.out.println("letting worker finish");
      worker.waitForResult(100000);

      System.out.println("result");
      System.out.println("Worker finished with a state = " + worker.getCurrentState());
      System.out.println("Worker final result = " + worker.getCurrentResult());

      // timeout
      System.out.println("----------------------------------------------");
      System.out.println("creating worker");
      worker = new DebugInterruptableWorker();

      System.out.println("resetting worker");
      worker.reset(0, 555, 5);

      System.out.println("starting worker");
      worker.startWorkOnANewThread();

      System.out.println("letting worker finish");
      worker.waitForResult(1000);

      System.out.println("result");
      System.out.println("Worker finished with a state = " + worker.getCurrentState());
      System.out.println("Worker final result = " + worker.getCurrentResult());

      // interruption
      System.out.println("----------------------------------------------");
      System.out.println("resetting worker");
      worker.reset(0, 100, 1);

      System.out.println("starting worker");
      worker.startWorkOnANewThread();

      System.out.println("wait a bit");

      while (((Integer) worker.getCurrentResult()) < 40)
      {
         System.out.println("result=" + worker.getCurrentResult());

         try
         {
            Thread.sleep(300);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

      System.out.println("interrupt");
      long t = System.currentTimeMillis();
      worker.stopWork();
      System.out.println("interrupt took " + (System.currentTimeMillis() - t));

      System.out.println("result");
      System.out.println("Worker finished with a state = " + worker.getCurrentState());
      System.out.println("Worker final result = " + worker.getCurrentResult());

      // try to finish before interruption to test thread safety
      System.out.println("----------------------------------------------");
      System.out.println("resetting worker");
      worker.reset(0, 1, 1);

      System.out.println("starting worker");
      worker.startWorkOnANewThread();

      Thread thread = new Thread(new SetState(worker));
      thread.setDaemon(true);
      thread.start();

      System.out.println("interrupt");
      t = System.currentTimeMillis();
      worker.stopWork();
      System.out.println("interrupt took " + (System.currentTimeMillis() - t));

      System.out.println("result");
      System.out.println("Worker finished with a state = " + worker.getCurrentState());
      System.out.println("Worker final result = " + worker.getCurrentResult());
   }

   static class SetState implements Runnable
   {
      private InterruptableWorker worker;

      public SetState(InterruptableWorker worker)
      {
         this.worker = worker;
      }

      @Override
      public void run()
      {
         worker.setCurrentState(STATE.INTERRUPTED);

         try
         {
            Thread.sleep(100);
         }
         catch (Exception e)
         {
         }
      }
   }
}
