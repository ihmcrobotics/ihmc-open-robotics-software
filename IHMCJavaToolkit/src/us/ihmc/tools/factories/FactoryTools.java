package us.ihmc.tools.factories;

import java.lang.reflect.Field;

public class FactoryTools
{
   public static void checkAllRequiredFactoryFieldsAreSet(Object factory)
   {
      for (Field field : factory.getClass().getDeclaredFields())
      {
         if (field.getType().equals(RequiredFactoryField.class))
         {
            RequiredFactoryField<?> requiredFactoryField = null;
            try
            {
               field.setAccessible(true);
               requiredFactoryField = (RequiredFactoryField<?>) field.get(factory);
               requiredFactoryField.get();
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               e.printStackTrace();
            }
         }
      }
   }
}
