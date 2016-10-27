package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;

import java.util.HashMap;

public class QuadrupedStepStreamMultiplexer<E extends Enum<E>> implements QuadrupedStepStream
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final HashMap<E, QuadrupedStepStream> stepStreams = new HashMap<>();
   private EnumYoVariable<E> selectedStepStream;

   public QuadrupedStepStreamMultiplexer(Class<E> enumClass, YoVariableRegistry parentRegistry)
   {
      selectedStepStream = new EnumYoVariable<>("selectedStepStream", registry, enumClass);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
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
      selectedStepStream.set(enumValue);
      return (stepStreams.get(enumValue) != null);
   }

   @Override
   public void onEntry()
   {
      QuadrupedStepStream stepStream = stepStreams.get(selectedStepStream.getEnumValue());
      if (stepStream != null)
      {
         stepStream.onEntry();
      }
   }

   @Override
   public void process()
   {
      QuadrupedStepStream stepStream = stepStreams.get(selectedStepStream.getEnumValue());
      if (stepStream != null)
      {
         stepStream.process();
      }
   }

   @Override
   public void onExit()
   {
      QuadrupedStepStream stepStream = stepStreams.get(selectedStepStream.getEnumValue());
      if (stepStream != null)
      {
         stepStream.onExit();
      }
   }

   @Override
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      QuadrupedStepStream stepStream = stepStreams.get(selectedStepStream.getEnumValue());
      if (stepStream != null)
      {
         return stepStream.getSteps();
      }
      else
      {
         throw new RuntimeException("A valid step stream must be selected prior to accessing step stream data.");
      }
   }

   @Override
   public void getBodyOrientation(FrameOrientation bodyOrientation)
   {
      QuadrupedStepStream stepStream = stepStreams.get(selectedStepStream.getEnumValue());
      if (stepStream != null)
      {
         stepStream.getBodyOrientation(bodyOrientation);
      }
      else
      {
         throw new RuntimeException("A valid step stream must be selected prior to accessing step stream data.");
      }
   }
}
