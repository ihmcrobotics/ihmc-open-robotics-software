package us.ihmc.gdx.simulation.environment.object;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.function.Supplier;

public class GDXSCS2EnvironmentObjectFactory
{
   private final String name;
   private final Class<? extends GDXSCS2EnvironmentObject> clazz;
   private Supplier<GDXSCS2EnvironmentObject> supplier;

   public GDXSCS2EnvironmentObjectFactory(String name, Class<? extends GDXSCS2EnvironmentObject> clazz)
   {
      this.name = name;
      this.clazz = clazz;

      try
      {
         Constructor<? extends GDXSCS2EnvironmentObject> constructor = clazz.getConstructor();
         supplier = () ->
         {
            GDXSCS2EnvironmentObject environmentObject = null;
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

   public Supplier<GDXSCS2EnvironmentObject> getSupplier()
   {
      return supplier;
   }

   public String getName()
   {
      return name;
   }

   public Class<? extends GDXSCS2EnvironmentObject> getClazz()
   {
      return clazz;
   }
}
