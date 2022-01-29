package us.ihmc.gdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;

public class GDXDoorFrameObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Frame";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorFrameObject.class);

   public GDXDoorFrameObject()
   {
      super(NAME, FACTORY);
      setRealisticModel(GDXModelLoader.loadG3DModel("environmentObjects/door/doorFrame/DoorFrame.g3dj"));
      double sizeX = 0.3;
      double sizeY = 0.3;
      double sizeZ = 0.01; // TODO: Fix dimensions
      setMass(100.0f);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      getBoundingSphere().setRadius(0.7);
   }
}
