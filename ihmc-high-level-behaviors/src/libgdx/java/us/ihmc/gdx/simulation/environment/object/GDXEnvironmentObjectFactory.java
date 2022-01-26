package us.ihmc.gdx.simulation.environment.object;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.function.Supplier;

public class GDXEnvironmentObjectFactory
{
   private final String name;
   private final Class<? extends GDXEnvironmentObject> clazz;
   private Supplier<GDXEnvironmentObject> supplier;

   public GDXEnvironmentObjectFactory(String name, Class<? extends GDXEnvironmentObject> clazz)
   {
      this.name = name;
      this.clazz = clazz;

      try
      {
         Constructor<? extends GDXEnvironmentObject> constructor = clazz.getConstructor();
         supplier = () ->
         {
            GDXEnvironmentObject environmentObject = null;
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

   public Supplier<GDXEnvironmentObject> getSupplier()
   {
      return supplier;
   }

   public String getName()
   {
      return name;
   }

   public Class<? extends GDXEnvironmentObject> getClazz()
   {
      return clazz;
   }
}
