package us.ihmc.robotics.dataStructures.variable;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class LongYoVariable extends YoVariable<LongYoVariable>
{
   private long val;

   public LongYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   public LongYoVariable(String name, String description, YoVariableRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   public LongYoVariable(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.LONG, name, description, registry);

      this.set(0);
   }

   /**
    * NOT VERIFIED::::
    * 
    * Set the value of this YoVariable.  All four types are represented via the same interal double.  If
    * of integer type this value is cast as an int whenever accessed.  It represents the ordinal if of enum type.
    * Boolean values are triggered around the 0.5 threshold with < 0.5 being false and greater or equal being true.
    *
    * @param value double value to store
    */
   public void set(long value)
   {
      set(value, true);
   }
   
   public boolean set(long value, boolean notifyListeners)
   {
      if (val != value)
      {
         val = value;
         if(notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }
      return false;
   }
   
   public void increment()
   {
      this.set(this.getLongValue()+1);
   }
   
   public void decrement()
   {
      this.set(this.getLongValue()-1);
   }
   
   public void add(long value)
   {
      this.set(this.getLongValue() + value);
   }
   
   public void subtract(long value)
   {
      this.set(this.getLongValue() - value);
   }

   /**
    * Retrieve the integer representation of this variable.
    *
    * @return int value of this
    */
   public long getLongValue()
   {
      return val;
   }

   /**
    * Check if the value contained by this variable is equal to the given integer.  If not of integer type
    * a warning will be printed to the console.
    *
    * @param value int to be compared
    * @return boolean are they equal?
    */
   public boolean valueEquals(long value)
   {
      return val == value;
   }

   @Override
   public void setValueFromDouble(double doubleValue, boolean notifyListeners)
   {
      set(convertFromDoubleToLong(doubleValue), notifyListeners);
   }
   
   public long convertFromDoubleToLong(double doubleValue)
   {
      return Math.round(doubleValue);
   }

   @Override
   public double getValueAsDouble()
   {
      return val;
   }

   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();
      retBuffer.append(getName());
      retBuffer.append(": ");
      retBuffer.append(getLongValue());
      return retBuffer.toString();
   }

   /**
    * Appends the value of this variable to the end of the given StringBuffer.  This representation is
    * based on variable type.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override
   public void getValueString(StringBuffer stringBuffer)
   {
      stringBuffer.append(val);
   }
   
   @Override
   public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      stringBuffer.append(convertFromDoubleToLong(doubleValue));
   }

   @Override
   public long getValueAsLongBits()
   {
      return val;
   }

   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(value, notifyListeners);
   }

   @Override
   public LongYoVariable duplicate(YoVariableRegistry newRegistry)
   {
      LongYoVariable retVar = new LongYoVariable(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(getLongValue());
      return retVar;
   }

   @Override
   public boolean setValue(LongYoVariable value, boolean notifyListeners)
   {
      return set(value.getLongValue(), notifyListeners);
   }

   @Override
   public boolean isZero()
   {
      return getLongValue() == 0l;
   }
}
