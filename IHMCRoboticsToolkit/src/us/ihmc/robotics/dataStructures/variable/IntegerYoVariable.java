package us.ihmc.robotics.dataStructures.variable;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class IntegerYoVariable extends YoVariable<IntegerYoVariable>
{
   private int val;

   public IntegerYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   public IntegerYoVariable(String name, String description, YoVariableRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   public IntegerYoVariable(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.INTEGER, name, description, registry);

      this.set(0);
   }

   /**
    * Set the value of this YoVariable.  All four types are represented via the same interal double.  If
    * of integer type this value is cast as an int whenever accessed.  It represents the ordinal if of enum type.
    * Boolean values are triggered around the 0.5 threshold with < 0.5 being false and greater or equal being true.
    *
    * @param value double value to store
    */
   public void set(int value)
   {
      set(value, true);
   }
   
   public boolean set(int value, boolean notifyListeners)
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
      this.set(this.getIntegerValue()+1);
   }
   
   public void decrement()
   {
      this.set(this.getIntegerValue()-1);
   }
   
   public void add(int value)
   {
      this.set(this.getIntegerValue() + value);
   }
   
   public void subtract(int value)
   {
      this.set(this.getIntegerValue() - value);
   }

   /**
    * Retrieve the integer representation of this variable.  If not of integer type a warning will be printed.
    *
    * @return int value of this
    */
   public int getIntegerValue()
   {
//      if (val != Math.round(val))
//         System.err.println("IntegerYoVariable: returning a different value than what it used to be.");
      return val;
   }

   /**
    * Check if the value contained by this variable is equal to the given integer.  If not of integer type
    * a warning will be printed to the console.
    *
    * @param value int to be compared
    * @return boolean are they equal?
    */
   public boolean valueEquals(int value)
   {
      return val == value;
   }

   @Override
   public void setValueFromDouble(double doubleValue, boolean notifyListeners)
   {
      set(convertFromDoubleToInt(doubleValue), notifyListeners);
   }
   
   public int convertFromDoubleToInt(double doubleValue)
   {
      // Note: do not expect this to work well for very large values!
      return (int) Math.round(doubleValue);
      //   return (new Double(doubleValue).intValue());
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
      retBuffer.append(getIntegerValue());
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
      stringBuffer.append(convertFromDoubleToInt(doubleValue));
   }

   @Override
   public long getValueAsLongBits()
   {
      return val;
   }

   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set((int) value, notifyListeners);
   }

   @Override
   public IntegerYoVariable duplicate(YoVariableRegistry newRegistry)
   {
      IntegerYoVariable retVar = new IntegerYoVariable(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(getIntegerValue());
      return retVar;
   }

   @Override
   public boolean setValue(IntegerYoVariable value, boolean notifyListeners)
   {
      return set(value.getIntegerValue(), notifyListeners);
   }

   @Override
   public boolean isZero()
   {
      return getIntegerValue() == 0;
   }

}
