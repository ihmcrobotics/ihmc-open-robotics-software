package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXLabFloorObject extends RDXEnvironmentObject
{
   public static final String NAME = "Lab Floor";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXLabFloorObject.class);

   public RDXLabFloorObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/labFloor/LabFloor.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 6.0;
      double sizeY = 6.0;
      double sizeZ = 0.01;
      setMass(10000.0f);
      getBoundingSphere().setRadius(0.7);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
   }

   @Override
   public void setSelected(boolean selected)
   {
      // (NK) Do nothing
   }
}
