package us.ihmc.simulationConstructionSetTools.util.globalParameters;

import us.ihmc.yoVariables.variable.YoEnum;

public class EnumGlobalParameter extends GlobalParameter
{
   public <T extends Enum<T>> EnumGlobalParameter(String name, String description, T value, GlobalParameterChangedListener listener)
   {
      super(null, listener);

      YoEnum<T> yoEnum = new YoEnum<T>(name, description, registry, value.getDeclaringClass(), false);
      yoEnum.set(value);
      this.yoVariable = yoEnum;

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   public <T extends Enum<T>> EnumGlobalParameter(String name, String description, Class<T> enumType, GlobalParameter[] parents, GlobalParameterChangedListener listener)
   {
      super(parents, listener);

      yoVariable = YoEnum.create(name, description, enumType, registry, false);

      if (changedListener != null)
         changedListener.globalParameterCreated(this);
   }

   public Enum<?> getValue()
   {
      return ((YoEnum<?>)yoVariable).getEnumValue();
   }

   protected int getNumberOfCharactersForDisplay()
   {
      Enum<?>[] enumValues = ((Enum<?>) ((YoEnum<?>)yoVariable).getEnumValue()).getDeclaringClass().getEnumConstants();
      int maxNumberOfCharacters = Integer.MIN_VALUE;

      for (Enum<?> enumValue : enumValues)
      {
         if (enumValue.toString().length() > maxNumberOfCharacters)
            maxNumberOfCharacters = enumValue.toString().length();
      }

      return maxNumberOfCharacters;
   }

   public <T extends Enum<T>> void set(T value)
   {
      set(value, "");
   }

   public <T extends Enum<T>> void set(T value, String comment)
   {
      verifyNoParents();
      setEnumValue(value, comment);
   }

   public <T extends Enum<T>> void setOnlyIfChange(T value)
   {
      setOnlyIfChange(value, "");
   }

   public <T extends Enum<T>> void setOnlyIfChange(T value, String comment)
   {
      verifyNoParents();

      // check if the value is the same as the current value
      if (value == ((YoEnum<?>)yoVariable).getEnumValue())
         return;
      else
         setEnumValue(value, comment);
   }

   @Override
   public String getValueInStringFormat()
   {
      String ret = ((YoEnum<?>)yoVariable).getEnumValue().toString();

      return padWithSpaces(ret, numberOfCharactersForDisplay);
   }


   @SuppressWarnings("unchecked")
   private <T extends Enum<T>> void setEnumValue(T newValue, String comment)
   {
      Enum<?> previousValue = ((YoEnum<?>)yoVariable).getEnumValue();
      ((YoEnum<T>)yoVariable).set(newValue);

      if (changedListener != null)
      {
         changedListener.enumValueChanged(this, comment, previousValue, newValue);
      }

      updateChildren(comment);
   }


}
