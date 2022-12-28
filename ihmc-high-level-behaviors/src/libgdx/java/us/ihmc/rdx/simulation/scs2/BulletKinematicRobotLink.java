package us.ihmc.rdx.simulation.scs2;

import org.bytedeco.bullet.LinearMath.btVector3;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTerrainObject;

public class BulletKinematicRobotLink
{
   private final Object referenceFrameAccessSync = new Object();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final BulletTerrainObject bulletTerrainObject;
   private final ReferenceFrame frameAfterJoint;

   public BulletKinematicRobotLink(ReferenceFrame frameAfterJoint,
                                   BulletPhysicsEngine bulletPhysicsEngine,
                                   TerrainObjectDefinition terrainObjectDefinition)
   {
      this.frameAfterJoint = frameAfterJoint;
      bulletTerrainObject = bulletPhysicsEngine.addAndGetTerrainObject(terrainObjectDefinition);
      bulletTerrainObject.getBtRigidBody().setMassProps(1.0, new btVector3(0.5, 0.5, 0.5));
   }

   public void beforePhysics(double time)
   {
      synchronized (referenceFrameAccessSync)
      {
         bulletTerrainObject.setTransformToWorld(transformToWorld);
      }
   }

   public void update()
   {
      synchronized (referenceFrameAccessSync)
      {
         frameAfterJoint.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
      }
   }
}
