package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkFlags;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXTools;

public class GDXDoorCombinedMultiBodyObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Combined Multi Body";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorCombinedMultiBodyObject.class);
   private final GDXDoorFrameObject doorFrameObject;
   private final GDXDoorPanelObject doorPanelObject;
   private final GDXDoorLeverHandleObject doorLeverObject;
   private final Matrix4 tempGDXTransform = new Matrix4();

   private btMultiBody multiBody;

   public GDXDoorCombinedMultiBodyObject()
   {
      super(NAME, FACTORY);

      doorFrameObject = new GDXDoorFrameObject();
      doorPanelObject = new GDXDoorPanelObject();
      doorLeverObject = new GDXDoorLeverHandleObject();
   }

   @Override
   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {
      int numberOfLinks = 2;
      float mass = 0.0f;
      Vector3 inertia = new Vector3();
//      doorFrameObject.getInertia(inertia);
      boolean fixedBase = false;
      boolean canSleep = false;
      multiBody = new btMultiBody(numberOfLinks, mass, inertia, fixedBase, canSleep);
      multiBody.setBasePos(new Vector3());
      multiBody.setWorldToBaseRot(new Quaternion().idt());

      int linkIndex = 0;
      mass = doorFrameObject.getMass();
      inertia = new Vector3();
      doorPanelObject.getInertia(inertia);
      int parentIndex = -1;
      Quaternion rotationFromParent = new Quaternion();
      Vector3 offsetOfPivotFromParentCenterOfMass = new Vector3(0.0f, 0.0f, 0.0f);
      Vector3 offsetOfCenterOfMassFromPivot = new Vector3(0.0f, 0.0f, 0.0f);
      multiBody.setupFixed(linkIndex,
                           mass,
                           inertia,
                           parentIndex,
                           rotationFromParent,
                           offsetOfPivotFromParentCenterOfMass,
                           offsetOfCenterOfMassFromPivot);

      linkIndex = 1;
      mass = doorPanelObject.getMass();
      inertia = new Vector3();
      doorPanelObject.getInertia(inertia);
      parentIndex = 0;
      rotationFromParent = new Quaternion();
      Vector3 jointAxis = new Vector3(0.0f, 0.0f, 1.0f);
      offsetOfPivotFromParentCenterOfMass = new Vector3(0.0f, 0.03f, 0.01f);
      offsetOfCenterOfMassFromPivot = new Vector3(0.0f, 0.9144f / 2.0f, 2.0447f / 2.0f);
      boolean disableParentCollision = false;
      multiBody.setupRevolute(linkIndex,
                              mass,
                              inertia,
                              parentIndex,
                              rotationFromParent,
                              jointAxis,
                              offsetOfPivotFromParentCenterOfMass,
                              offsetOfCenterOfMassFromPivot,
                              disableParentCollision);

//      multiBody.getLink(0).setFlags(multiBody.getLink(0).getFlags() & ~btMultiBodyLinkFlags.BT_MULTIBODYLINKFLAGS_DISABLE_ALL_PARENT_COLLISION);
//      multiBody.getLink(0).setFlags(multiBody.getLink(0).getFlags() & ~btMultiBodyLinkFlags.BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION);

      multiBody.finalizeMultiDof();
      bulletPhysicsManager.addMultiBody(multiBody);

//      multiBody.setJointPos(0, 0.0f);

      btMultiBodyLinkCollider frameCollider = new btMultiBodyLinkCollider(multiBody, -1);
      frameCollider.setCollisionShape(doorFrameObject.getBtCollisionShape());
      Matrix4 worldTransform = new Matrix4();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toGDX(rigidBodyTransform, worldTransform);
      frameCollider.setWorldTransform(worldTransform);
      bulletPhysicsManager.getMultiBodyDynamicsWorld().addCollisionObject(frameCollider); // TODO: Store and remove

      multiBody.setBaseCollider(frameCollider);
//      multiBody.getLink(0).setCollider(frameCollider);

      btMultiBodyLinkCollider panelCollider = new btMultiBodyLinkCollider(multiBody, 1);
      panelCollider.setCollisionShape(doorPanelObject.getBtCollisionShape());
      panelCollider.setWorldTransform(worldTransform);
      bulletPhysicsManager.getMultiBodyDynamicsWorld().addCollisionObject(panelCollider);

      multiBody.getLink(1).setCollider(panelCollider);

   }

   @Override
   public void copyBulletTransformToThisMultiBody()
   {
      doorFrameObject.copyBulletTransformToThis(multiBody.getBaseCollider().getWorldTransform());
      doorPanelObject.copyBulletTransformToThis(multiBody.getLink(1).getCollider().getWorldTransform());
   }

   @Override
   public void copyThisTransformToBulletMultiBody()
   {
      doorFrameObject.getThisTransformForCopyToBullet(tempGDXTransform);
      multiBody.setBaseWorldTransform(tempGDXTransform);
   }

   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      return doorFrameObject.intersect(pickRay, intersectionToPack);
   }

   @Override
   public void setSelected(boolean selected)
   {
      doorFrameObject.setSelected(selected);
   }

   @Override
   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      doorFrameObject.setPositionInWorld(positionInWorld);
   }

   @Override
   public void setPoseInWorld(Pose3D poseInWorld)
   {
      doorFrameObject.setPoseInWorld(poseInWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      doorFrameObject.setTransformToWorld(transformToWorld);
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      doorFrameObject.getRealRenderables(renderables, pool);
      doorPanelObject.getRealRenderables(renderables, pool);
      doorLeverObject.getRealRenderables(renderables, pool);
   }

   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      doorFrameObject.getCollisionMeshRenderables(renderables, pool);
      doorPanelObject.getCollisionMeshRenderables(renderables, pool);
      doorLeverObject.getCollisionMeshRenderables(renderables, pool);
   }

   @Override
   public RigidBodyTransform getObjectTransform()
   {
      return doorFrameObject.getObjectTransform();
   }

   @Override
   public boolean getIsSelected()
   {
      return doorFrameObject.getIsSelected();
   }
}
