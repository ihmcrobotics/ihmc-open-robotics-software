package us.ihmc.rdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btHingeConstraint;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsManager;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXRockHingeObject extends RDXEnvironmentObject
{
   public static final String NAME = "Rock Hinge";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXMultiBodySnakeObject.class);
   private final RDXRockObject rockParentObject;
   private final RDXRockObject rockChildObject;
   private btHingeConstraint rockHingeConstraint;

   public RDXRockHingeObject()
   {
      super(NAME, FACTORY);

      rockParentObject = new RDXRockObject(40, 0.5, 0.3, 0f, 8121);
      rockChildObject = new RDXRockObject(40, 0.5, 0.3, 900f, 5214);
   }

   @Override
   public void addToBullet(RDXBulletPhysicsManager bulletPhysicsManager)
   {
      rockParentObject.addToBullet(bulletPhysicsManager);
      rockChildObject.addToBullet(bulletPhysicsManager);
      rockParentObject.getBtRigidBody().setActivationState(4);
      rockChildObject.getBtRigidBody().setActivationState(4);

      Vector3 axisA = new Vector3(0.0f, 0.2f, 0.0f); 
      Vector3 axisB = new Vector3(0.0f, 0.0f, 0.0f); 
      Vector3 pivotA = new Vector3(-0.2f, 0.0f, -0.2f);
      Vector3 pivotB = new Vector3(0.2f, 0.0f, 0.2f);

      rockHingeConstraint = new btHingeConstraint(rockParentObject.getBtRigidBody(), rockChildObject.getBtRigidBody(), pivotA, pivotB, axisA, axisB, true);

      rockHingeConstraint.setLimit(-1.0f, 1.0f);
      addConstraint(bulletPhysicsManager, rockHingeConstraint);
   }

   @Override
   public void removeFromBullet()
   {
      super.removeFromBullet();
      rockHingeConstraint.dispose();
      rockParentObject.removeFromBullet();
      rockChildObject.removeFromBullet();

   }

   @Override
   public void updateRenderablesPoses()
   {
      // do nothing
   }

   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      return rockParentObject.intersect(pickRay, intersectionToPack);
   }

   @Override
   public void setSelected(boolean selected)
   {
      rockParentObject.setSelected(selected);
   }

   @Override
   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      rockParentObject.setPositionInWorld(positionInWorld);
      rockChildObject.setPositionInWorld(new Point3D(positionInWorld.getX(), positionInWorld.getY(), positionInWorld.getZ() - 0.5));
   }

   @Override
   public void setPoseInWorld(Pose3D poseInWorld)
   {
      rockParentObject.setPoseInWorld(poseInWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      rockParentObject.setTransformToWorld(transformToWorld);
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      rockParentObject.getRealRenderables(renderables, pool);
      rockChildObject.getRealRenderables(renderables, pool);
   }

   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      rockParentObject.getCollisionMeshRenderables(renderables, pool);
      rockChildObject.getCollisionMeshRenderables(renderables, pool);
   }

   @Override
   public RigidBodyTransform getObjectTransform()
   {
      return rockParentObject.getObjectTransform();
   }

}
