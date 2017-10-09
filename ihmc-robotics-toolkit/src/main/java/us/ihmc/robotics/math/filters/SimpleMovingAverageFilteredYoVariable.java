package us.ihmc.robotics.math.filters;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 *  Filter the given yoVariable using a moving average filter.
 *
 *
 *  This class is NOT REWINDABLE!
 *
 */
public class SimpleMovingAverageFilteredYoVariable extends YoDouble
{
   private final YoInteger windowSize;
   private final YoDouble yoVariableToFilter;

   private final DenseMatrix64F previousUpdateValues = new DenseMatrix64F(0, 0);
   private int bufferPosition = 0;

   private boolean bufferHasBeenFilled = false;

   public SimpleMovingAverageFilteredYoVariable(String name, int windowSize, YoVariableRegistry registry)
   {
      this(name, windowSize, null, registry);
   }

   public SimpleMovingAverageFilteredYoVariable(String name, int windowSize, YoDouble yoVariableToFilter, YoVariableRegistry registry)
   {
      super(name, registry);

      this.yoVariableToFilter = yoVariableToFilter;
      this.windowSize = new YoInteger(name + "WindowSize", registry);
      this.windowSize.set(windowSize);

      previousUpdateValues.reshape(windowSize, 1);
      CommonOps.fill(previousUpdateValues, 0.0);
   }

   public void setWindowSize(int windowSize)
   {
      this.windowSize.set(windowSize);
      reset();
   }

   public void update()
   {
      update(yoVariableToFilter.getDoubleValue());
   }

   public void update(double value)
   {
      if (previousUpdateValues.getNumRows() != windowSize.getIntegerValue())
      {
         reset();
      }
      previousUpdateValues.set(bufferPosition, 0, value);

      bufferPosition++;

      if (bufferPosition >= windowSize.getIntegerValue())
      {
         bufferPosition = 0;
         bufferHasBeenFilled = true;
      }

      double average = 0;
      for (int i = 0; i < windowSize.getIntegerValue(); i++)
      {
         average += previousUpdateValues.get(i, 0);
      }

      this.set(average / windowSize.getIntegerValue());
   }

   public void reset()
   {
      bufferPosition = 0;
      bufferHasBeenFilled = false;
      previousUpdateValues.reshape(windowSize.getIntegerValue(), 1);
   }

   public boolean getHasBufferWindowFilled()
   {
      return bufferHasBeenFilled;
   }
}
