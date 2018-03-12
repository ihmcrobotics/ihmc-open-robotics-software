package us.ihmc.robotics.allocations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.google.monitoring.runtime.instrumentation.AllocationRecorder;
import com.google.monitoring.runtime.instrumentation.Sampler;

public class AllocationSampler implements Sampler
{
   private final Set<String> methodsToIgnore = new HashSet<>();
   private final Set<String> methodsToWatch = new HashSet<>();
   private final Set<String> classesToIgnore = new HashSet<>();
   private final Set<String> classesToWatch = new HashSet<>();

   private final Queue<Throwable> allocations = new ConcurrentLinkedQueue<>();

   public AllocationSampler()
   {
      addClassToIgnore(AllocationRecorder.class.getName());
   }

   public void addBlacklistMethod(String methodName)
   {
      methodsToIgnore.add(methodName);
   }

   public void addMethodToWatch(String methodName)
   {
      methodsToWatch.add(methodName);
   }

   public void addClassToIgnore(String className)
   {
      classesToIgnore.add(className);
   }

   public void addClassToWatch(String className)
   {
      classesToWatch.add(className);
   }

   public List<Throwable> getAndClearAllocations()
   {
      List<Throwable> ret = new ArrayList<>();
      Throwable throwable;
      while ((throwable = allocations.poll()) != null)
      {
         ret.add(throwable);
      }
      return ret;
   }

   @Override
   public void sampleAllocation(int count, String desc, Object newObj, long size)
   {
      StackTraceElement[] stackTrace = getCleanedStackTace();

      if (!checkIfOfInterest(stackTrace))
      {
         return;
      }
      if (checkIfBlacklisted(stackTrace))
      {
         return;
      }

      // Skip static member initializations.
      if (stackTrace[0].getMethodName().contains("<clinit>"))
      {
         return;
      }

      Throwable throwable = new Throwable("Allocation of Object: " + newObj.getClass().getSimpleName());
      throwable.setStackTrace(stackTrace);
      allocations.add(throwable);
   }

   private boolean checkIfOfInterest(StackTraceElement[] stackTrace)
   {
      if (methodsToWatch.isEmpty() && classesToWatch.isEmpty())
      {
         return true;
      }

      for (StackTraceElement el : stackTrace)
      {
         String qualifiedMethodName = el.getClassName() + "." + el.getMethodName();
         if (methodsToWatch.contains(qualifiedMethodName))
         {
            return true;
         }
         for (String packet : classesToWatch)
         {
            if (el.getClassName().startsWith(packet))
            {
               return true;
            }
         }
      }
      return false;
   }

   private boolean checkIfBlacklisted(StackTraceElement[] stackTrace)
   {
      if (methodsToIgnore.isEmpty() && classesToIgnore.isEmpty())
      {
         return false;
      }

      for (StackTraceElement el : stackTrace)
      {
         String qualifiedMethodName = el.getClassName() + "." + el.getMethodName();
         if (methodsToIgnore.contains(qualifiedMethodName))
         {
            return true;
         }
         for (String packet : classesToIgnore)
         {
            if (el.getClassName().startsWith(packet))
            {
               return true;
            }
         }
      }
      return false;
   }

   private StackTraceElement[] getCleanedStackTace()
   {
      String recordClass = AllocationRecorder.class.getName();
      List<StackTraceElement> stackTrace = Arrays.asList(Thread.currentThread().getStackTrace());
      int skip = 0;
      while (!stackTrace.get(skip).toString().contains(recordClass))
      {
         skip++;
      }
      while (stackTrace.get(skip).toString().contains(recordClass))
      {
         skip++;
      }
      return stackTrace.subList(skip, stackTrace.size()).toArray(new StackTraceElement[0]);
   }
}
