package us.ihmc.rdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.physics.bullet.collision.*;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXDoorLeverHandleObject extends RDXEnvironmentObject
{
   public static final String NAME = "Door Lever Handle";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXDoorLeverHandleObject.class);

   public RDXDoorLeverHandleObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/door/doorLeverHandle/DoorLeverHandle.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 0.065;
      double sizeY = 0.14;
      double sizeZ = 0.065;
      setMass(0.7f);
      getCollisionShapeOffset().getTranslation().add(-sizeX / 2.0, -0.04, 0.0);
      getBoundingSphere().setRadius(0.2);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);

      btTriangleIndexVertexArray btTriangleIndexVertexArray = new btTriangleIndexVertexArray(realisticModel.meshParts.get(0));
      btGImpactMeshShape btGImpactMeshShape = new btGImpactMeshShape(btTriangleIndexVertexArray);
      btGImpactMeshShape.updateBound();
      setBtCollisionShape(btGImpactMeshShape);
   }
}
