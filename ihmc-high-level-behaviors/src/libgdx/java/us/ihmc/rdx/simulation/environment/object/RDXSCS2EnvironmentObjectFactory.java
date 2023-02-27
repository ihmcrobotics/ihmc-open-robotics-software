package us.ihmc.rdx.simulation.environment.object;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.function.Supplier;

public class RDXSCS2EnvironmentObjectFactory
{
   private final String name;
   private final Class<? extends RDXSCS2EnvironmentObject> clazz;
   private Supplier<RDXSCS2EnvironmentObject> supplier;

   public RDXSCS2EnvironmentObjectFactory(String name, Class<? extends RDXSCS2EnvironmentObject> clazz)
   {
      this.name = name;
      this.clazz = clazz;

      try
      {
         Constructor<? extends RDXSCS2EnvironmentObject> constructor = clazz.getConstructor();
         supplier = () ->
         {
            RDXSCS2EnvironmentObject environmentObject = null;
            try
            {
               environmentObject = constructor.newInstance();
               if (environmentObject == null)
               {
                  throw new RuntimeException("Why is this null?");
               }
            }
            catch (InstantiationException | IllegalAccessException | InvocationTargetException e)
            {
               e.printStackTrace();
            }


            return environmentObject;
         };
      }
      catch (NoSuchMethodException e)
      {
         e.printStackTrace();
      }
   }

   public Supplier<RDXSCS2EnvironmentObject> getSupplier()
   {
      return supplier;
   }

   public String getName()
   {
      return name;
   }

   public Class<? extends RDXSCS2EnvironmentObject> getClazz()
   {
      return clazz;
   }
}
