package us.ihmc.robotics.allocations;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * This is a demo for the {@link AllocationSampler}.
 * <p>
 * For this to work you will need to add the following argument to your java VM arguments:<br>
 * {@code -javaagent:[your/path/to/]java-allocation-instrumenter-3.1.0.jar}<br>
 * </p>
 * @author Georg
 */
public class AllocationRecordingDemo implements AllocationTest
{
   public AllocationRecordingDemo()
   {
      int initialSize = 6;
      RecyclingArrayList<Vector3D> myList = new RecyclingArrayList<>(initialSize, Vector3D.class);

      // This should not allocate objects since the list is large enough.
      List<Throwable> allocations = runAndCollectAllocations(() -> {
         myList.clear();
         for (int i = 0; i < initialSize; i++)
         {
            myList.add();
         }
      });
      PrintTools.info("Number of places where allocations occured: " + allocations.size());
      allocations.forEach(allocation -> allocation.printStackTrace());

      // This should allocate a new Vector3D
      allocations = runAndCollectAllocations(() -> {
         myList.add();
      });
      PrintTools.info("Number of places where allocations occured: " + allocations.size());

      allocations.forEach(allocation -> allocation.printStackTrace());
   }

   @Override
   public List<Class<?>> getClassesOfInterest()
   {
      return Collections.singletonList(RecyclingArrayList.class);
   }

   @Override
   public List<Class<?>> getClassesToIgnore()
   {
      return Collections.emptyList();
   }

   public static void main(String[] args) throws IOException
   {
      new AllocationRecordingDemo();
   }
}
