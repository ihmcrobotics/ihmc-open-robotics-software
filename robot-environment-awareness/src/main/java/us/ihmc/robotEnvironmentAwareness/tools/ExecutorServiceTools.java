package us.ihmc.robotEnvironmentAwareness.tools;

import java.util.concurrent.Callable;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;

public class ExecutorServiceTools
{
   public enum ExceptionHandling
   {
      NONE(false, false), CANCEL_AND_REPORT(false, true), CATCH(true, false), CATCH_AND_REPORT(true, true);

      private final boolean catchExceptions;
      private final boolean reportExceptions;

      private ExceptionHandling(boolean catchExceptions, boolean reportExceptions)
      {
         this.catchExceptions = catchExceptions;
         this.reportExceptions = reportExceptions;
      }

      private boolean isCatchExceptionsEnabled()
      {
         return catchExceptions;
      }

      private boolean shouldReportExceptions()
      {
         return reportExceptions;
      }
   };

   public static ScheduledExecutorService newSingleThreadScheduledExecutor(Class<?> creator, ExceptionHandling exceptionHandling)
   {
      return newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(creator.getSimpleName()), exceptionHandling);
   }

   public static ScheduledExecutorService newSingleThreadScheduledExecutor(ThreadFactory threadFactory, ExceptionHandling exceptionHandling)
   {
      return newScheduledThreadPool(1, threadFactory, exceptionHandling);
   }

   public static ScheduledExecutorService newScheduledThreadPool(int corePoolSize, Class<?> creator, ExceptionHandling exceptionHandling)
   {
      return newScheduledThreadPool(corePoolSize, ThreadTools.getNamedThreadFactory(creator.getSimpleName()), exceptionHandling);
   }

   public static ScheduledExecutorService newScheduledThreadPool(int corePoolSize, ThreadFactory threadFactory, ExceptionHandling exceptionHandling)
   {
      return new ScheduledThreadPoolExecutor(corePoolSize, threadFactory)
      {
         @Override
         public ScheduledFuture<?> schedule(Runnable command, long delay, TimeUnit unit)
         {
            return super.schedule(wrapWithTryAndCatch(command, exceptionHandling), delay, unit);
         }

         @Override
         public <V> ScheduledFuture<V> schedule(Callable<V> callable, long delay, TimeUnit unit)
         {
            return super.schedule(wrapWithTryAndCatch(callable, exceptionHandling), delay, unit);
         }

         @Override
         public ScheduledFuture<?> scheduleAtFixedRate(Runnable command, long initialDelay, long period, TimeUnit unit)
         {
            return super.scheduleAtFixedRate(wrapWithTryAndCatch(command, exceptionHandling), initialDelay, period, unit);
         }

         public ScheduledFuture<?> scheduleWithFixedDelay(Runnable command, long initialDelay, long delay, TimeUnit unit)
         {
            return super.scheduleWithFixedDelay(wrapWithTryAndCatch(command, exceptionHandling), initialDelay, delay, unit);
         }
      };
   }

   private static <V> Callable<V> wrapWithTryAndCatch(Callable<V> callable, ExceptionHandling exceptionHandling)
   {
      return new Callable<V>()
      {
         @Override
         public V call() throws Exception
         {
            try
            {
               return callable.call();
            }
            catch (Exception e)
            {
               if (exceptionHandling.shouldReportExceptions())
                  e.printStackTrace();

               if (!exceptionHandling.isCatchExceptionsEnabled())
               {
                  PrintTools.warn(ExecutorServiceTools.class, "Current task is cancelled.");
                  throw e;
               }

               return null;
            }
         }
      };
   }

   private static Runnable wrapWithTryAndCatch(Runnable command, ExceptionHandling exceptionHandling)
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               command.run();
            }
            catch (Exception e)
            {
               if (exceptionHandling.shouldReportExceptions())
                  e.printStackTrace();

               if (!exceptionHandling.isCatchExceptionsEnabled())
               {
                  PrintTools.warn(ExecutorServiceTools.class, "Current task is cancelled.");
                  throw e;
               }
            }
         }
      };
   }

}
