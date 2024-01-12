package us.ihmc.rdx.simulation.environment.object;

import us.ihmc.rdx.simulation.environment.object.objects.*;
import us.ihmc.rdx.simulation.environment.object.objects.door.*;

import java.util.ArrayList;

public class RDXEnvironmentObjectLibrary
{
   private static final ArrayList<RDXEnvironmentObjectFactory> objectFactories = new ArrayList<>();
   static
   {
      objectFactories.add(RDXSmallCinderBlockRoughed.FACTORY);
      objectFactories.add(RDXMediumCinderBlockRoughed.FACTORY);
      objectFactories.add(RDXLargeCinderBlockRoughed.FACTORY);
      objectFactories.add(RDXLabFloorObject.FACTORY);
      objectFactories.add(RDXArUcoBoxObject.FACTORY);
      objectFactories.add(RDXL515SensorObject.FACTORY);
   }

   public static ArrayList<RDXEnvironmentObjectFactory> getObjectFactories()
   {
      return objectFactories;
   }

   public static RDXEnvironmentObject loadBySimpleClassName(String objectClassName)
   {
      for (RDXEnvironmentObjectFactory objectFactory : objectFactories)
      {
         if (objectFactory.getClazz().getSimpleName().equals(objectClassName))
         {
            return objectFactory.getSupplier().get();
         }
      }

      throw new RuntimeException("Library does not contain object of the name: " + objectClassName);
   }
}
