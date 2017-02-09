package us.ihmc.robotics.dataStructures.variable;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class BooleanYoVariable extends YoVariable<BooleanYoVariable>
{
   private boolean val;

   public BooleanYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   public BooleanYoVariable(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.BOOLEAN, name, description, registry);

      this.stepSize = 1.0;
      this.set(false);
   }

   /**
    * Check if the value contained by this variable is equal to the given double.  If not of double type
    * a warning will be printed to the console.
    *
    * @param value boolean to be compared to
    * @return are they equal?
    */
   public boolean valueEquals(boolean value)
   {
      return (this.val == value);
   }

   /**
    * Retrieve the boolean value of this YoVariable, if not of boolean type a warning will be printed.
    * @return boolean value of this
    */
   public boolean getBooleanValue()
   {
      return val;
   }

   /**
    * Sets the boolean value of this YoVariable.  If not of boolean type a warning will be printed.
    * @param value boolean
    */
   public void set(boolean value)
   {
      set(value, true);
   }

   public boolean set(boolean value, boolean notifyListeners)
   {
      if (val != value)
      {
         this.val = value;
         if (notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }
      return false;
   }

   @Override
   public void setValueFromDouble(double value, boolean notifyListeners)
   {
      set(convertDoubleToBoolean(value), notifyListeners);
   }

   public boolean convertDoubleToBoolean(double value)
   {
      if (value >= 0.5)
         return true;
      else
         return false;
   }

   @Override
   public double getValueAsDouble()
   {
      double returnValue = 0.0;

      if (getBooleanValue())
         returnValue = 1.0;
      else
         returnValue = 0.0;

      return returnValue;
   }

   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();
      retBuffer.append(getName());
      retBuffer.append(": ");
      retBuffer.append(getBooleanValue());
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
      stringBuffer.append(convertDoubleToBoolean(doubleValue));
   }

   @Override
   public long getValueAsLongBits()
   {
      return getBooleanValue() ? 1 : 0;
   }

   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(value == 1, notifyListeners);
   }

   @Override
   public BooleanYoVariable duplicate(YoVariableRegistry newRegistry)
   {
      BooleanYoVariable newVar = new BooleanYoVariable(getName(), getDescription(), newRegistry);
      newVar.set(getBooleanValue());
      return newVar;
   }

   @Override
   public boolean setValue(BooleanYoVariable value, boolean notifyListeners)
   {
      return set(value.getBooleanValue(), notifyListeners);
   }

   @Override
   public boolean isZero()
   {
      return !getBooleanValue();
   }
   
   
}