package us.ihmc.simulationconstructionset.util;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class IndexOrderChecker
{
   private final IntegerYoVariable previousIndex;
   private final IntegerYoVariable missedIndices;
   private final BooleanYoVariable hasBeenUpdated;
   private int increment;
   
   public IndexOrderChecker(String name, YoVariableRegistry registry, int increment)
   {
      previousIndex = new IntegerYoVariable(name + "PrevIndex", registry);
      missedIndices = new IntegerYoVariable(name + "MissedIndices", registry);
      hasBeenUpdated = new BooleanYoVariable(name + "HasBeenUpdated", registry);
      this.increment = increment;
   }
   
   public void update(int newIndex)
   {
      if (hasBeenUpdated.getBooleanValue())
      {
         int correctedNew = newIndex;
         int correctedPrevious = previousIndex.getIntegerValue();
         if (correctedPrevious > correctedNew)
         {
            // assume a single overflow
            correctedNew += Integer.MIN_VALUE;
            correctedPrevious -= Integer.MAX_VALUE + 1;
         }
         missedIndices.add(Math.abs(correctedNew - correctedPrevious - increment)); // abs because we also don't want the new index to be too small
      }
      else
         hasBeenUpdated.set(true);
      previousIndex.set(newIndex);
   }

   public int getMissedIndices()
   {
      return missedIndices.getIntegerValue();
   }
}
