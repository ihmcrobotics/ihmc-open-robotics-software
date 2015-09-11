package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class SimpleMovingAverageFilteredYoVariable extends DoubleYoVariable
{
   private int windowSize;

   private final DoubleYoVariable yoVariableToFilter;


   private final double previousUpdateValues[];
   private int bufferPosition = 0;

   /**
    *  Filter the given yoVariable using a moving average filter.
    *
    *
    *  This class is NOT REWINDABLE!
    *
    */
   
   public SimpleMovingAverageFilteredYoVariable(String name, int windowSize, YoVariableRegistry registry)
   {
      this(name, windowSize, null, registry);
   }
   
   public SimpleMovingAverageFilteredYoVariable(String name, int windowSize, DoubleYoVariable yoVariableToFilter, YoVariableRegistry registry)
   {
      super(name, registry);

      this.yoVariableToFilter = yoVariableToFilter;
      this.windowSize = windowSize;

      previousUpdateValues = new double[windowSize];

      for (int i = 0; i < windowSize; i++)
      {
         previousUpdateValues[i] = 0.0;
      }

   }
   
   public void update()
   {
      update(yoVariableToFilter.getDoubleValue());
   }

   boolean bufferHasBeenFilled = false;
   
   public void update(double value)
   {
      previousUpdateValues[bufferPosition] = value;

      bufferPosition++;

      if (bufferPosition >= windowSize)
      {
         bufferPosition = 0;
         bufferHasBeenFilled = true;
      }

      double average = 0;
      for (int i = 0; i < windowSize; i++)
      {
         average += previousUpdateValues[i];
      }

      this.set(average / windowSize);
   }
   
   public void reset()
   {
      bufferHasBeenFilled = false;
   }
   
   public boolean getHasBufferWindowFilled()
   {
      return bufferHasBeenFilled;
   }


   public static void main(String[] args)
   {
      DoubleYoVariable testVar = new DoubleYoVariable("Test", null);

      SimpleMovingAverageFilteredYoVariable filter = new SimpleMovingAverageFilteredYoVariable("test_filtered", 100, testVar, null);

      for (int i = 0; i < 200; i++)
      {
         testVar.set(1);
         filter.update();
         System.out.println(filter.getDoubleValue());

      }
      
      for (int i = 0; i < 200; i++)
      {
         testVar.set(0);
         filter.update();
         System.out.println(filter.getDoubleValue());

      }

      System.out.println(filter.getDoubleValue());



   }



}
