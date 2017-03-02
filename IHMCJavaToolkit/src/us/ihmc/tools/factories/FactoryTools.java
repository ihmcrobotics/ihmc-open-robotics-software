package us.ihmc.tools.factories;

import java.lang.reflect.Field;

public class FactoryTools
{
   public static void checkAllFactoryFieldsAreSet(Object factory)
   {
      for (Field field : factory.getClass().getDeclaredFields())
      {
         if (FactoryField.class.isAssignableFrom(field.getType()))
         {
            try
            {
               field.setAccessible(true);
               
               if (OptionalFactoryField.class.isAssignableFrom(field.getType()))
               {
                  OptionalFactoryField<?> optionalFactoryField = (OptionalFactoryField<?>) field.get(factory);
                  if (optionalFactoryField.hasValue())
                  {
                     optionalFactoryField.get();
                  }
               }
               else
               {
                  RequiredFactoryField<?> requiredFactoryField = (RequiredFactoryField<?>) field.get(factory);
                  requiredFactoryField.get();
               }
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               e.printStackTrace();
            }
         }
      }
   }
   
   public static void disposeFactory(Object factory)
   {
      for (Field field : factory.getClass().getDeclaredFields())
      {
         if (FactoryField.class.isAssignableFrom(field.getType()))
         {
            FactoryField<?> factoryField = null;
            try
            {
               field.setAccessible(true);
               factoryField = (FactoryField<?>) field.get(factory);
               factoryField.dispose();
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               e.printStackTrace();
            }
         }
      }
   }
}
