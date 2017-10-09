package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>Title: </p>
 *
 * <p>Description: applies hysteresis to the input </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 * @author tkoolen
 * @version 1.0
 */
public class HysteresisFilteredYoVariable extends YoDouble
{
   private YoDouble hysteresisAmount;
   private double previousUnfilteredValue;
   private double upperLimit, lowerLimit;

   public HysteresisFilteredYoVariable(String name, YoVariableRegistry registry, YoDouble hysteresisAmount)
   {
      super(name, registry);
      this.hysteresisAmount = hysteresisAmount;
      this.set(Double.NaN);
   }

   /**
    * reset
    * resets the limits to the current value (but keeps the current hysteresisAmount)
    */
   public void reset()
   {
      rangeCheck(hysteresisAmount.getDoubleValue());
      super.set(previousUnfilteredValue);
      upperLimit = getDoubleValue() + hysteresisAmount.getDoubleValue() / 2.0;
      lowerLimit = getDoubleValue() - hysteresisAmount.getDoubleValue() / 2.0;
   }

   /**
    * update
    * updates val, based on the unfiltedValue
    *
    * @param unfilteredValue double the unfiltered value
    */
   public void update(double unfilteredValue)
   {
      rangeCheck(hysteresisAmount.getDoubleValue());
      previousUnfilteredValue = unfilteredValue;

      if (Double.isNaN(getDoubleValue()))
      {
         // initialization
         super.set(unfilteredValue);

         upperLimit = getDoubleValue() + hysteresisAmount.getDoubleValue() / 2.0;
         lowerLimit = getDoubleValue() - hysteresisAmount.getDoubleValue() / 2.0;
      }
      else
      {
         boolean exceededUpper = unfilteredValue > upperLimit;
         boolean exceededLower = unfilteredValue < lowerLimit;

         if (exceededUpper)
         {
            upperLimit = unfilteredValue;
            lowerLimit = upperLimit - hysteresisAmount.getDoubleValue();

            super.set((upperLimit + lowerLimit) / 2.0);
         }

         else if (exceededLower)
         {
            lowerLimit = unfilteredValue;
            upperLimit = lowerLimit + hysteresisAmount.getDoubleValue();

            super.set((upperLimit + lowerLimit) / 2.0);
         }
      }
   }

   /**
    * rangeCheck standard NaN and sign checks
    *
    * @param hysteresisAmount double
    */
   private void rangeCheck(double hysteresisAmount)
   {
      if (Double.isNaN(hysteresisAmount))
      {
         throw new RuntimeException("hysteresisAmount is NaN");
      }

      if (hysteresisAmount < 0.0)
      {
         throw new RuntimeException("hysteresisAmount is negative");
      }
   }

   public static void main(String[] args)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble hysteresisAmount = new YoDouble("hysteresisAmount", registry);
      new HysteresisFilteredYoVariable("test", registry, hysteresisAmount);

      System.out.println("done");
   }
}
