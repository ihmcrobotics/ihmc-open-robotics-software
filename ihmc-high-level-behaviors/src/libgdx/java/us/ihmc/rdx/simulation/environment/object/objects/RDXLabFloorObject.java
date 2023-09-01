package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsManager;
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

      double sizeX = 0.3;
      double sizeY = 0.3;
      double sizeZ = 0.01;
      setMass(10000.0f);
      getBoundingSphere().setRadius(0.7);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionGeometryObject(collisionBox);
      setBtCollisionShape(new btBoxShape(new Vector3(10.0f, 10.0f, 1.0f)));
      getBulletCollisionOffset().getTranslation().addZ(-1.0);
   }

   @Override
   public void addToBullet(RDXBulletPhysicsManager bulletPhysicsManager)
   {
      super.addToBullet(bulletPhysicsManager);
      bulletPhysicsManager.setKinematicObject(getBtRigidBody(), true); // has to be a massive kinematics object to collide with multi bodies
   }

   @Override
   public void setSelected(boolean selected)
   {
      setRawIsSelected(selected);
   }
}
