package us.ihmc.robotics.dataStructures.variable;

import java.util.Arrays;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class EnumYoVariable<T extends Enum<T>> extends YoVariable<EnumYoVariable<T>>
{
   public static final int NULL_VALUE = -1;

   private Class<T> enumType;
   private final boolean allowNullValue;
   private T[] enumValues;
   private String[] enumValuesAsString;

   private int valueOrdinal;

   /**
    * shortcut construction method; infers the type parameter from the declared type
    */
   public static <T extends Enum<T>> EnumYoVariable<T> create(String name, Class<T> enumType, YoVariableRegistry registry)
   {
      return new EnumYoVariable<>(name, registry, enumType);
   }

   /**
    * shortcut construction method; infers the type parameter from the declared type
    */
   public static <T extends Enum<T>> EnumYoVariable<T> create(String name, String description, Class<T> enumType, YoVariableRegistry registry, boolean allowNullValue)
   {
      return new EnumYoVariable<>(name, description, registry, enumType, allowNullValue);
   }

   /**
    * Creates a new YoVariable with the given name and type and adds it to the specified registry.  This
    * constructor allows the user to provide a simple description of the variable, which can
    * come in handy when faced with thousands.  The description is displayed in the VarPanel
    * component of the GUI.  This variant also allows the user to specify min and max values for
    * manual scaling of graphs.
    *
    * @param name name to be used for all references of this variable by SCS
    * @param description a short description
    * @param registry YoVariableRegistry
    * @param enumType the class representing the type of the enum
    */
   public EnumYoVariable(String name, String description, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValue)
   {
      super(YoVariableType.ENUM, name, description, registry);

      this.enumType = enumType;
      this.allowNullValue = allowNullValue;
      this.enumValues = enumType.getEnumConstants();

      enumValuesAsString = new String[enumValues.length];
      for(int i = 0; i < enumValues.length; i++)
      {
         enumValuesAsString[i] = enumValues[i].toString();
      }
      
      set(0, true);
   }

   /**
    * Create a new EnumYoVariable based on Strings. 
    * 
    * <p>
    * When a new EnumYoVariable has to be created based on data from an external source, this constructor can be used.
    * The main use case is the LogVisualizer and the SCSVisualizer code. Do not use this constructor in controls code.
    * </p>
    * 
    * <p>
    * The functions working on Enums directly will throw UnsupportedOperationException when this constructor is used.
    * </p>
    * 
    * 
    * @param name the name of the YoVariable
    * @param description the short description of the YoVariable
    * @param registry the registry to add this variable to
    * @param values array of values that this enum can take
    */
   public EnumYoVariable(String name, String description, YoVariableRegistry registry, boolean allowNullValues, String... values)
   {
      super(YoVariableType.ENUM, name, description, registry);

      this.enumType = null;
      this.allowNullValue = allowNullValues;
      this.enumValues = null;
      
      enumValuesAsString = Arrays.copyOf(values, values.length);
      set(0, true);
   }

   public EnumYoVariable(String name, YoVariableRegistry registry, Class<T> enumType)
   {
      this(name, "", registry, enumType, false);
   }

   public EnumYoVariable(String name, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValue)
   {
      this(name, "", registry, enumType, allowNullValue);
   }

   public boolean isBackedByEnum()
   {
      return enumType != null;
   }
   
   private void checkIfBackedByEnum()
   {
      if (enumType == null)
      {
         throw new UnsupportedOperationException("This EnumYoVariable is not backed by an Enum variable.");
      }
   }
   
   /**
    * Check if the value contained by this variable is equal to the given enum.  If this YoVariable
    * is not of enum type a warning will be printed to the console.
    *
    * @param value Enum to be compared
    * @return boolean
    */
   public boolean valueEquals(T value)
   {
      checkIfBackedByEnum();
      if (valueOrdinal == NULL_VALUE)
         return value == null;

      return (value.ordinal() == valueOrdinal);
   }

   public Class<T> getEnumType()
   {
      checkIfBackedByEnum();
      return enumType;
   }
   
   public void setEnumType(Class<T> enumType)
   {
      this.enumType = enumType;
      enumValues = enumType.getEnumConstants();
      enumValuesAsString = new String[enumValues.length];
      for(int i = 0; i < enumValues.length; i++)
      {
         enumValuesAsString[i] = enumValues[i].toString();
      }
   }

   /**
    * Set the enum value of this variable.  If not of enum type a warning will be printed.
    *
    * @param enumValue Enum to store in this YoVariable
    */
   public void set(T enumValue)
   {
      set(enumValue, true);
   }
   
   public void set(int ordinal)
   {
      set(ordinal, true);
   }
   
   public boolean set(T enumValue, boolean notifyListeners)
   {
      checkIfBackedByEnum();
      if (!allowNullValue && (enumValue == null))
      {
         throw new RuntimeException("Setting EnumYoVariable " + getName()
         + " to null. Must set allowNullValue to true in the constructor if you ever want to set it to null.");
      }
      
      return set(enumValue == null ? NULL_VALUE : enumValue.ordinal(), notifyListeners);
   }
      
   public boolean set(int ordinal, boolean notifyListeners)
   {
      checkBounds(ordinal);
      
      if (valueOrdinal != NULL_VALUE)
      {
         if (!(valueOrdinal == ordinal))
         {
            valueOrdinal = ordinal;
            if (notifyListeners)
            {
               notifyVariableChangedListeners();
            }
            return true;
         }
      }
      else
      {
         valueOrdinal = ordinal;
         if (notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }

      return false;

   }

   private void checkBounds(int ordinal)
   {
      if((ordinal < 0 && !(allowNullValue && ordinal == NULL_VALUE))  || ordinal >= enumValuesAsString.length)
      {
         throw new RuntimeException("Enum constant associated with value " + ordinal + " not present. VariableName = " + this.getFullNameWithNameSpace());
      }
   }

   public boolean getAllowNullValue()
   {
      return allowNullValue;
   }

   public T[] getEnumValues()
   {
      checkIfBackedByEnum();
      return enumValues;
   }
   
   public String[] getEnumValuesAsString()
   {
      return enumValuesAsString;
   }

   /**
    * Retrieve the enum value of this YoVariable.  If this variable is not of enum type a warning will be printed.
    *
    * @return Enum value of this
    */
   public T getEnumValue()
   {
      checkIfBackedByEnum();
      return valueOrdinal == NULL_VALUE? null : enumValues[valueOrdinal];
   }

   public String getStringValue()
   {
      if(valueOrdinal == NULL_VALUE)
      {
         return "null";
      }
      else
      {
         return enumValuesAsString[valueOrdinal];
      }
   }
   
   @Override
   public void setValueFromDouble(double value, boolean notifyListeners)
   {
      try
      {
         int index = (int) Math.round(value);
         set(index, notifyListeners);
      }
      catch (RuntimeException ignored)
      {
         
      }
   }

   @Override
   public double getValueAsDouble()
   {
      return valueOrdinal;
   }

   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();
      retBuffer.append(getName());
      retBuffer.append(": ");
      retBuffer.append(getStringValue());
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
      stringBuffer.append(getStringValue());
   }

   @Override
   public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      int index = (int) Math.round(doubleValue);
      checkBounds(index);
      if(index == NULL_VALUE)
      {
         stringBuffer.append("null");
      }
      else
      {
         stringBuffer.append(enumValuesAsString[index]);
      }
   }

   @Override
   public long getValueAsLongBits()
   {
      return valueOrdinal;
   }

   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set((int) value, notifyListeners);
   }

   @Override
   public EnumYoVariable<T> duplicate(YoVariableRegistry newRegistry)
   {
      EnumYoVariable<T> retVar = new EnumYoVariable<>(getName(), getDescription(), newRegistry, getEnumType(), getAllowNullValue());
      retVar.set(getEnumValue());
      return retVar;
   }

   @Override
   public boolean setValue(EnumYoVariable<T> value, boolean notifyListeners)
   {
      return set(value.getEnumValue(), notifyListeners);
   }
   
   public int getOrdinal()
   {
      return valueOrdinal;
   }
   
   public int getEnumSize()
   {
      return enumValuesAsString.length;
   }

   @Override
   public boolean isZero()
   {
      return getEnumValue() == null;
   }
}