package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.geometry.FrameOrientation;

import java.util.HashMap;

public class QuadrupedStepStreamMultiplexer<E extends Enum<E>> implements QuadrupedStepStream
{
   private final HashMap<E, QuadrupedStepStream> stepStreams = new HashMap<>();
   private QuadrupedStepStream selectedStepStream = null;

   public QuadrupedStepStreamMultiplexer()
   {
   }

   /**
    * Add a new step stream (does not modify active step stream).
    * @param enumValue step stream enum value
    * @param instance step stream instance
    */
   public void addStepStream(E enumValue, QuadrupedStepStream instance)
   {
      stepStreams.put(enumValue, instance);
   }

   /**
    * Select the active step stream.
    * @param enumValue step stream enum value
    * @return true if selected step stream is valid
    */
   public boolean selectStepStream(E enumValue)
   {
      selectedStepStream = stepStreams.get(enumValue);
      return (selectedStepStream != null);
   }

   @Override
   public void onEntry()
   {
      if (selectedStepStream != null)
      {
         selectedStepStream.onEntry();
      }
   }

   @Override
   public void process()
   {
      if (selectedStepStream != null)
      {
         selectedStepStream.process();
      }
   }

   @Override
   public void onExit()
   {
      if (selectedStepStream != null)
      {
         selectedStepStream.onExit();
      }
   }

   @Override
   public PreallocatedQueue<QuadrupedTimedStep> getSteps()
   {
      if (selectedStepStream != null)
      {
         return selectedStepStream.getSteps();
      }
      else
      {
         throw new RuntimeException("A valid step stream must be selected prior to accessing step stream data.");
      }
   }

   @Override
   public void getBodyOrientation(FrameOrientation bodyOrientation)
   {
      if (selectedStepStream != null)
      {
         selectedStepStream.getBodyOrientation(bodyOrientation);
      }
      else
      {
         throw new RuntimeException("A valid step stream must be selected prior to accessing step stream data.");
      }
   }
}
