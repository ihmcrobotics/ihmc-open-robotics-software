package us.ihmc.rdx.simulation.environment.object;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.function.Supplier;

public class RDXEnvironmentObjectFactory
{
   private final String name;
   private final Class<? extends RDXEnvironmentObject> clazz;
   private Supplier<RDXEnvironmentObject> supplier;

   public RDXEnvironmentObjectFactory(String name, Class<? extends RDXEnvironmentObject> clazz)
   {
      this.name = name;
      this.clazz = clazz;

      try
      {
         Constructor<? extends RDXEnvironmentObject> constructor = clazz.getConstructor();
         supplier = () ->
         {
            RDXEnvironmentObject environmentObject = null;
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

   public Supplier<RDXEnvironmentObject> getSupplier()
   {
      return supplier;
   }

   public String getName()
   {
      return name;
   }

   public Class<? extends RDXEnvironmentObject> getClazz()
   {
      return clazz;
   }
}
