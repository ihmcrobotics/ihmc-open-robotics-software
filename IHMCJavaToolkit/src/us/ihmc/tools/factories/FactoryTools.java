package us.ihmc.tools.factories;

import java.lang.reflect.Field;

public class FactoryTools
{
   public static void checkAllRequiredFactoryFieldsAreSet(Factory factory)
   {
      for (Field field : factory.getClass().getDeclaredFields())
      {
         if (field.getType().equals(RequiredFactoryField.class))
         {
            RequiredFactoryField<?> requiredFactoryField = null;
            try
            {
               requiredFactoryField = (RequiredFactoryField<?>) factory.accessFieldValue(field, factory);
               
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
