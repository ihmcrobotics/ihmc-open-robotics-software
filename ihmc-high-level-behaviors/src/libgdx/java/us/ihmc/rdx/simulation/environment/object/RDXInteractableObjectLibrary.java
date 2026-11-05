package us.ihmc.rdx.simulation.environment.object;

import us.ihmc.rdx.simulation.environment.object.objects.RDXArUcoBoxObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXBottleObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXCanObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXCerealBoxObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXChargeObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXDrillObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXShoeObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXTableObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXTrashCanObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXTrowelObject;

import java.util.Set;

/**
 * Objects MuJoCo simulates as free bodies, so the robot can push or grasp them.
 * Anything not registered here stays mocap: contact still works, but the object does not fall or get pushed.
 */
public class RDXInteractableObjectLibrary
{
   private static final Set<Class<? extends RDXEnvironmentObject>> interactableObjects = Set.of(
         RDXBottleObject.class,
         RDXChargeObject.class,
         RDXDrillObject.class,
         RDXTrowelObject.class,
         RDXArUcoBoxObject.class,
         RDXCanObject.class,
         RDXCerealBoxObject.class,
         RDXShoeObject.class,
         RDXTrashCanObject.class,
         RDXTableObject.class);

   public static Set<Class<? extends RDXEnvironmentObject>> getInteractableObjects()
   {
      return interactableObjects;
   }

   public static boolean isInteractable(RDXEnvironmentObject object)
   {
      if (object == null)
         return false;
      Class<?> type = object.getClass();
      while (RDXEnvironmentObject.class.isAssignableFrom(type))
      {
         if (interactableObjects.contains(type))
            return true;
         type = type.getSuperclass();
      }
      return false;
   }
}
