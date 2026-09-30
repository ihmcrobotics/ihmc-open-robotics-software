package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXLabFloorObject extends RDXEnvironmentObject
{
   public static final String NAME = "Lab Floor";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLabFloorObject.class);

   public RDXLabFloorObject()
   {
      super(NAME, FACTORY);
      loadRealisticModel("environmentObjects/labFloor/LabFloor.g3dj");

      double sizeX = 0.3;
      double sizeY = 0.3;
      double sizeZ = 0.01;
      setMass(10000.0f);
      getBoundingSphere().setRadius(0.7);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
//      getBulletCollisionOffset().getTranslation().addZ(-1.0);
   }

   @Override
   public void setSelected(boolean selected)
   {
      setRawIsSelected(selected);
   }
}
