package us.ihmc.rdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.tools.RDXModelLoader;

public class RDXDoorPanelObject extends RDXEnvironmentObject
{
   public static final String NAME = "Door Panel";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXDoorPanelObject.class);

   public RDXDoorPanelObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = RDXModelLoader.load("environmentObjects/door/doorPanel/DoorPanelWithFiducials.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 0.0508; // centered on X
      double sizeY = 0.9144;
      double sizeZ = 2.0447;
      getCenterOfMassInModelFrame().set(0.0, sizeY / 2.0, sizeZ / 2.0);
      double simulationY = sizeY - 0.05; // prevent door hinge self collision
      setMass(70.0f);
      getRealisticModelOffset().getTranslation().sub(getCenterOfMassInModelFrame());
      getCollisionShapeOffset().getTranslation().sub(getCenterOfMassInModelFrame());
      getCollisionShapeOffset().getTranslation().add(0.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      getBoundingSphere().setRadius(2.5);
      getBoundingSphere().getPosition().sub(getCenterOfMassInModelFrame());
      getBulletCollisionOffset().getTranslation().sub(getCenterOfMassInModelFrame());
      getBulletCollisionOffset().getTranslation().add(0.0, sizeY / 2.0 + 0.025, sizeZ / 2.0);

      setBtCollisionShape(new btBoxShape(new Vector3((float) sizeX / 2.0f, (float) simulationY / 2.0f, (float) sizeZ / 2.0f)));
   }
}
