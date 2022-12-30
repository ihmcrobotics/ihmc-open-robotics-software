package us.ihmc.rdx.simulation.scs2;

import org.bytedeco.bullet.LinearMath.btVector3;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletTerrainObject;

public class BulletKinematicRobotLink
{
   private final Object referenceFrameAccessSync = new Object();
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private BulletTerrainObject bulletTerrainObject;
   private final ReferenceFrame frameAfterJoint;
   private final BulletPhysicsEngine bulletPhysicsEngine;

   private final TerrainObjectDefinition terrainObjectDefinition = new TerrainObjectDefinition();
   private final MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.RosyBrown());
   private final RigidBodyTransform originPose = new RigidBodyTransform();

   public BulletKinematicRobotLink(ReferenceFrame frameAfterJoint, BulletPhysicsEngine bulletPhysicsEngine)
   {
      this.frameAfterJoint = frameAfterJoint;
      this.bulletPhysicsEngine = bulletPhysicsEngine;
   }

   public void setOriginPosition(Tuple3DReadOnly originPosition)
   {
      originPose.getTranslation().set(originPosition);

   }
   public void setOriginPosition(double x, double y, double z)
   {
      originPose.getTranslation().set(x, y, z);
   }

   public void setOriginYawPitchRoll(double yaw, double pitch, double roll)
   {
      originPose.getRotation().setYawPitchRoll(yaw, pitch, roll);
   }

   public void addCollidableShape(GeometryDefinition geometryDefinition)
   {
//      terrainObjectDefinition.addVisualDefinition(new VisualDefinition(originPose, geometryDefinition, materialDefinition));
      terrainObjectDefinition.addCollisionShapeDefinition(new CollisionShapeDefinition(originPose, geometryDefinition));
   }

   public void build()
   {
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
