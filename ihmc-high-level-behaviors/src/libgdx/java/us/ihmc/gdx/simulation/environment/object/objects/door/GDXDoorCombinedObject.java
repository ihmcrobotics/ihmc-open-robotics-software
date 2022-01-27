package us.ihmc.gdx.simulation.environment.object.objects.door;

import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;

public class GDXDoorCombinedObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Combined";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorCombinedObject.class);
   private final GDXDoorPanelObject doorPanelObject;
   private final GDXDoorLeverHandleObject doorLeverObject;

   public GDXDoorCombinedObject()
   {
      super(NAME, FACTORY);

      doorPanelObject = new GDXDoorPanelObject();
      doorLeverObject = new GDXDoorLeverHandleObject();
   }

   @Override
   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {

   }

   @Override
   public void removeFromBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {

   }
}
