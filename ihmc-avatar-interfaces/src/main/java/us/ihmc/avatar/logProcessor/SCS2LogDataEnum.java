package us.ihmc.avatar.logProcessor;

import us.ihmc.yoVariables.variable.YoEnum;

public class SCS2LogDataEnum<E extends Enum<E>>
{
   private final YoEnum<?> yoEnum;
   private final Class<E> enumType;
   private E lastValue = null;

   public SCS2LogDataEnum(YoEnum<?> yoEnum, Class<E> enumType)
   {
      this.yoEnum = yoEnum;
      this.enumType = enumType;
   }

   public E getValue()
   {
      for (E enumConstant : enumType.getEnumConstants())
      {
         if (yoEnum.getStringValue().equals(enumConstant.name()))
         {
            return enumConstant;
         }
      }

      return null;
   }

   public boolean changedFrom(E fromValue)
   {
      E currentValue = getValue();
      boolean changedFrom = lastValue == fromValue && currentValue != fromValue;
      lastValue = currentValue;
      return changedFrom;
   }

   public boolean changedTo(E toValue)
   {
      E currentValue = getValue();
      boolean changedTo = currentValue == toValue && lastValue != toValue;
      lastValue = currentValue;
      return changedTo;
   }
}
