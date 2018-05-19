package us.ihmc.robotics.allocations;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.exception.ExceptionUtils;

import com.google.monitoring.runtime.instrumentation.AllocationRecorder;

import us.ihmc.euclid.geometry.ConvexPolygon2D;

/**
 * This interface provides the means to profile the garbage generation of code. This is useful
 * in unit tests to check if the java code inside a controller is real time safe.
 * <p>
 * It does this using the {@link com.google.monitoring.runtime.instrumentation.AllocationRecorder}.
 * </p>
 * <p>
 * For an example of how to use this check {@link AllocationRecordingDemo}.
 * </p>
 * @author Georg
 */
public abstract interface AllocationTest
{
   /**
    * To avoid recording allocations that are of no interest it is possible to specify
    * classes of interest here. This avoids recording any garbage generated in other places
    * such as SCS. You should add you controller class here.
    *
    * @return classes that should be monitored for allocations.
    */
   public abstract List<Class<?>> getClassesOfInterest();

   /**
    * To avoid recording allocations that are of no interest it is possible to specify
    * classes that should be ignored here. This avoids recording any garbage generated in
    * places like the {@link ClassLoader} or in places that are simulation specific.
    *
    * @return classes that will be ignored when monitoring for allocations.
    */
   public abstract List<Class<?>> getClassesToIgnore();

   /**
    * To avoid recording allocations that are of no interest it is possible to specify
    * methods that should be ignored here. This avoids recording any garbage generated in
    * safe places like the {@link ConvexPolygon2D#setOrCreate} or in places that are
    * simulation specific.
    *
    * @return classes that will be ignored when monitoring for allocations.
    */
   public default List<String> getMethodsToIgnore()
   {
      return new ArrayList<>();
   }

   /**
    * Will run the provided runnable and return a list of places where allocations occurred.
    * If the returned list is empty no allocations where detected.
    *
    * @param runnable contains the code to be profiled.
    * @return a list of places where objects were allocated.
    */
   public default List<Throwable> runAndCollectAllocations(Runnable runnable)
   {
      checkInstrumentation();

      AllocationSampler sampler = new AllocationSampler();
      getClassesOfInterest().forEach(clazz -> sampler.addClassToWatch(clazz.getName()));
      getClassesToIgnore().forEach(clazz -> sampler.addClassToIgnore(clazz.getName()));
      getMethodsToIgnore().forEach(method -> sampler.addBlacklistMethod(method));

      AllocationRecorder.addSampler(sampler);
      runnable.run();
      sampler.stop();
      AllocationRecorder.removeSampler(sampler);

      return removeDuplicateStackTraces(sampler.getAndClearAllocations());
   }

   /**
    * This methos will check if the {@link AllocationRecorder} has an instrumentation. If this is not the case
    * the JVM was probably not started using the correct javaagent. To fix this start the JVM with the argument<br>
    * {@code -javaagent:[your/path/to/]java-allocation-instrumenter-3.1.0.jar}<br>
    *
    * @throws RuntimeException if no instrumentation exists or check for instrumentation failed.
    */
   public static void checkInstrumentation()
   {
      try
      {
         Method method = AllocationRecorder.class.getDeclaredMethod("getInstrumentation");
         method.setAccessible(true);
         if (method.invoke(null) == null)
         {
            throw new RuntimeException(AllocationRecorder.class.getSimpleName() + " has no instrumentation.");
         }
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Helper method to remove duplicate stack traces from a list of throwables.
    *
    * @param throwables list to prune of duplicate stack traces.
    * @return list of throwables with unique stack traces.
    */
   public static List<Throwable> removeDuplicateStackTraces(List<Throwable> throwables)
   {
      Map<String, Throwable> map = new HashMap<>();
      throwables.forEach(t -> map.put(ExceptionUtils.getStackTrace(t), t));
      return new ArrayList<>(map.values());
   }
}
