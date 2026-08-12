package us.ihmc.rdx.simulation.environment.object;

import us.ihmc.rdx.simulation.environment.object.objects.*;

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
      objectFactories.add(RDXPalletObject.FACTORY);
      objectFactories.add(RDXStairsObject.FACTORY);
      objectFactories.add(RDXWorkPlatformObject.FACTORY);
      objectFactories.add(RDXArUcoBoxObject.FACTORY);
      objectFactories.add(RDXPointLightObject.FACTORY);
      objectFactories.add(RDXPersonObject.FACTORY);
      objectFactories.add(RDXDoorPanelObject.FACTORY);
      objectFactories.add(RDXDoorLeverObject.FACTORY);
      objectFactories.add(RDXDirectionalLightObject.FACTORY);
      objectFactories.add(RDXL515SensorObject.FACTORY);
      objectFactories.add(RDXRightJerseyBarrierObject.FACTORY);
      objectFactories.add(RDXLeftJerseyBarrierObject.FACTORY);
      objectFactories.add(RDXCenteredJerseyBarrierObject.FACTORY);
      objectFactories.add(RDXChargeObject.FACTORY);
      objectFactories.add(RDXCouchObject.FACTORY);
      objectFactories.add(RDXTableObject.FACTORY);
      objectFactories.add(RDXTrashCanObject.FACTORY);
      objectFactories.add(RDXDrillObject.FACTORY);
      objectFactories.add(RDXShoeObject.FACTORY);
      objectFactories.add(RDXBikeObject.FACTORY);
      objectFactories.add(RDXBookObject.FACTORY);
      objectFactories.add(RDXMugObject.FACTORY);
      objectFactories.add(RDXCanObject.FACTORY);
      objectFactories.add(RDX2x4Object.FACTORY);
      objectFactories.add(RDXCerealBoxObject.FACTORY);
      objectFactories.add(RDXBottleObject.FACTORY);
      objectFactories.add(RDXBucketObject.FACTORY);
      objectFactories.add(RDXTrowelObject.FACTORY);
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
