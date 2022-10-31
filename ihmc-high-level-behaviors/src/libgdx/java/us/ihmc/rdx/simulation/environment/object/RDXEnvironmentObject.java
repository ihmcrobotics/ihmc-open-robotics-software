package us.ihmc.rdx.simulation.environment.object;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.dynamics.btTypedConstraint;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsManager;
import us.ihmc.rdx.tools.LibGDXTools;

import java.util.ArrayList;

/**
 * TODO: This class probably needs renaming and setup as layers of interfaces.
 */
public class RDXEnvironmentObject extends RDXSimpleObject
{
   private final Matrix4 tempGDXTransform = new Matrix4();
   private ReferenceFrame bulletCollisionFrame;
   private ReferenceFrame bulletCollisionSpecificationFrame;
   private final RigidBodyTransform bulletCollisionOffset = new RigidBodyTransform();
   private final RigidBodyTransform bulletCollisionFrameTransformToWorld = new RigidBodyTransform();
   private final FramePose3D bulletPose = new FramePose3D();
   private btCollisionShape btCollisionShape;
   private float mass = 0.0f;
   private final Point3D centerOfMassInModelFrame = new Point3D();
   private btRigidBody btRigidBody;
   private btMultiBody btMultiBody;
   private RDXBulletPhysicsManager bulletPhysicsManager;
   private boolean isSelected = false;
   private final ArrayList<btCollisionObject> addedCollisionObjects = new ArrayList<>();
   private final ArrayList<btTypedConstraint> addedConstraints = new ArrayList<>();
   public RDXEnvironmentObject(String titleCasedName, RDXEnvironmentObjectFactory factory)
   {
      super(titleCasedName, factory);
      bulletCollisionFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "BulletCollisionFrame" + objectIndex,
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              bulletCollisionFrameTransformToWorld);
      bulletCollisionSpecificationFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "BulletCollisionSpecificationFrame" + objectIndex,
                                                                              placementFrame,
                                                                              bulletCollisionOffset);
   }

   private final btMotionState bulletMotionState = new btMotionState()
   {
      @Override
      public void setWorldTransform(Matrix4 transformToWorld)
      {
         copyBulletTransformToThis(transformToWorld);
      }

      @Override
      public void getWorldTransform(Matrix4 transformToWorld)
      {
         getThisTransformForCopyToBullet(transformToWorld);
      }
   };

   public void copyBulletTransformToThisMultiBody()
   {
      if (btMultiBody != null)
         copyBulletTransformToThis(btMultiBody.getBaseWorldTransform());
   }

   public void update(RDXBulletPhysicsManager bulletPhysicsManager)
   {

   }

   public void afterSimulate(RDXBulletPhysicsManager bulletPhysicsManager)
   {

   }

   public void copyThisTransformToBulletMultiBody()
   {
      if (btMultiBody != null)
      {
         getThisTransformForCopyToBullet(tempGDXTransform);
         btMultiBody.setBaseWorldTransform(tempGDXTransform);
      }
   }

   public void copyThisTransformToBulletMultiBodyParentOnly()
   {
      if (btMultiBody != null)
      {
         getThisTransformForCopyToBullet(tempGDXTransform);
         btMultiBody.setBaseWorldTransform(tempGDXTransform);
      }
   }

   public void copyBulletTransformToThis(Matrix4 transformToWorld)
   {
      LibGDXTools.toEuclid(transformToWorld, bulletCollisionFrameTransformToWorld);
      bulletCollisionFrame.update();
      bulletPose.setToZero(bulletCollisionFrame);
      bulletPose.applyInverseTransform(bulletCollisionOffset);
      bulletPose.changeFrame(ReferenceFrame.getWorldFrame());
      bulletPose.get(tempTransform);
      setTransformToWorld(tempTransform);
   }

   public void getThisTransformForCopyToBullet(Matrix4 transformToWorld)
   {
      getThisTransformForCopyToBullet(tempTransform);
      LibGDXTools.toLibGDX(tempTransform, transformToWorld);
   }

   public void getThisTransformForCopyToBullet(RigidBodyTransform transformToWorld)
   {
      bulletCollisionSpecificationFrame.update();
      transformToWorld.set(bulletCollisionSpecificationFrame.getTransformToWorldFrame());
   }

   public void addToBullet(RDXBulletPhysicsManager bulletPhysicsManager)
   {
      this.bulletPhysicsManager = bulletPhysicsManager;
      if (btCollisionShape != null)
      {
         if (btRigidBody == null)
            btRigidBody = bulletPhysicsManager.addRigidBody(btCollisionShape, mass, getBulletMotionState());
      }
   }

   public void addBtMultiBody(RDXBulletPhysicsManager bulletPhysicsManager, btMultiBody btMultiBody)
   {
      this.bulletPhysicsManager = bulletPhysicsManager;
      this.btMultiBody = btMultiBody;
      bulletPhysicsManager.addMultiBody(btMultiBody);
   }

   public void addMultiBodyCollisionShape(RDXBulletPhysicsManager bulletPhysicsManager, btMultiBodyLinkCollider collisionObject)
   {
      this.bulletPhysicsManager = bulletPhysicsManager;
      addedCollisionObjects.add(collisionObject);
      bulletPhysicsManager.addMultiBodyCollisionShape(collisionObject);
   }

   public void addConstraint(RDXBulletPhysicsManager bulletPhysicsManager, btTypedConstraint constraint)
   {
      this.bulletPhysicsManager = bulletPhysicsManager;
      addedConstraints.add(constraint);
      bulletPhysicsManager.getMultiBodyDynamicsWorld().addConstraint(constraint);
   }

   public void getInertia(Vector3 inertiaToPack)
   {
      btCollisionShape.calculateLocalInertia(mass, inertiaToPack);
   }

   public void removeFromBullet()
   {
      if (btRigidBody != null)
      {
         bulletPhysicsManager.removeCollisionObject(btRigidBody);
      }
      if (btMultiBody != null)
      {
         bulletPhysicsManager.removeMultiBody(btMultiBody);
      }
      for (btCollisionObject addedCollisionObject : addedCollisionObjects)
      {
         bulletPhysicsManager.removeCollisionObject(addedCollisionObject);
      }
      addedCollisionObjects.clear();
      for (btTypedConstraint addedConstraint : addedConstraints)
      {
         bulletPhysicsManager.getMultiBodyDynamicsWorld().removeConstraint(addedConstraint);
      }
      addedConstraints.clear();
   }

   public RDXEnvironmentObject duplicate()
   {
      return factory.getSupplier().get();
   }


   public btMotionState getBulletMotionState()
   {
      return bulletMotionState;
   }

   public RigidBodyTransform getBulletCollisionOffset()
   {
      return bulletCollisionOffset;
   }

   public void setMass(float mass)
   {
      this.mass = mass;
   }

   public float getMass()
   {
      return mass;
   }

   public Point3D getCenterOfMassInModelFrame()
   {
      return centerOfMassInModelFrame;
   }

   public void setBtCollisionShape(btCollisionShape btCollisionShape)
   {
      this.btCollisionShape = btCollisionShape;
   }

   public com.badlogic.gdx.physics.bullet.collision.btCollisionShape getBtCollisionShape()
   {
      return btCollisionShape;
   }

   public btRigidBody getBtRigidBody()
   {
      return btRigidBody;
   }

   public ReferenceFrame getBulletCollisionSpecificationFrame()
   {
      return bulletCollisionSpecificationFrame;
   }

   public RDXBulletPhysicsManager getBulletPhysicsManager()
   {
      return bulletPhysicsManager;
   }

   public void setSelected(boolean selected)
   {
      isSelected = selected;
      if (btRigidBody != null)
      {
         bulletPhysicsManager.setKinematicObject(btRigidBody, selected);
      }
   }
}
