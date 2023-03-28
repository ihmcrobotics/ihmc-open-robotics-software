package us.ihmc.rdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btHingeConstraint;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImFloat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.simulation.bullet.RDXBulletPhysicsManager;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

public class RDXDoorCombinedObject extends RDXEnvironmentObject
{
   public static final String NAME = "Door Combined";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXDoorCombinedObject.class);
   private final RDXDoorFrameObject doorFrameObject;
   private final RDXDoorPanelObject doorPanelObject;
   private final RDXDoorLeverHandleObject doorLeverObject;

   private static final ImFloat lowLimit = new ImFloat(-0.5f);
   private static final ImFloat highLimit = new ImFloat(0.5f);
   private static final ImFloat relaxationFactor = new ImFloat(1.0f);
   private static final ImFloat biasFactor = new ImFloat(0.3f);
   private static final ImFloat softness = new ImFloat(0.9f);
   private btHingeConstraint doorHingeConstraint;
   private btHingeConstraint handleHingeConstraint;

   public RDXDoorCombinedObject()
   {
      super(NAME, FACTORY);

      doorFrameObject = new RDXDoorFrameObject();
      doorPanelObject = new RDXDoorPanelObject();
      doorLeverObject = new RDXDoorLeverHandleObject();
   }

   @Override
   public void addToBullet(RDXBulletPhysicsManager bulletPhysicsManager)
   {
      doorFrameObject.addToBullet(bulletPhysicsManager);
      doorPanelObject.addToBullet(bulletPhysicsManager);
      doorLeverObject.addToBullet(bulletPhysicsManager);

      Vector3 pivotInFrameForDoorHinge = new Vector3(0.0f, 0.0f, 0.01f); // get the bottom of the door off the ground a little
      Vector3 pivotInPanelForDoorHinge = new Vector3(0.0f, -((0.9144f - 0.05f) / 2.0f) - 0.05f, -2.0447f / 2.0f); // prevent door hinge self collision
      Vector3 axisInFrameForDoorHinge = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 axisInPanelForDoorHinge = new Vector3(0.0f, 0.0f, 1.0f);
      boolean useReferenceFrameAForDoorHinge = true;
      doorHingeConstraint = new btHingeConstraint(doorFrameObject.getBtRigidBody(),
                                                                    doorPanelObject.getBtRigidBody(),
                                                                    pivotInFrameForDoorHinge,
                                                                    pivotInPanelForDoorHinge,
                                                                    axisInFrameForDoorHinge,
                                                                    axisInPanelForDoorHinge,
                                                                    useReferenceFrameAForDoorHinge);
      doorHingeConstraint.setLimit(-2.0f, 2.0f, softness.get(), biasFactor.get(), relaxationFactor.get());
      addConstraint(bulletPhysicsManager, doorHingeConstraint);

//      Vector3 pivotInPanel = new Vector3(-0.03f, 0.85f, 0.9f);
      Vector3 pivotInPanelForHandle = new Vector3(-0.03f, 0.4f, -0.13f);
      Vector3 pivotInLeverForHandle = new Vector3(0.0f, 0.0f, 0.0f);
      Vector3 axisInPanelForHandle = new Vector3(1.0f, 0.0f, 0.0f);
      Vector3 axisInLeverForHandle = new Vector3(1.0f, 0.0f, 0.0f);
      boolean useReferenceFrameAForHandle = true;
      handleHingeConstraint = new btHingeConstraint(doorPanelObject.getBtRigidBody(),
                                                                      doorLeverObject.getBtRigidBody(),
                                                                      pivotInPanelForHandle,
                                                                      pivotInLeverForHandle,
                                                                      axisInPanelForHandle,
                                                                      axisInLeverForHandle,
                                                                      useReferenceFrameAForHandle);
      // these limits from 0.0 to 1.0?
      handleHingeConstraint.setLimit(lowLimit.get(), highLimit.get(), softness.get(), biasFactor.get(), relaxationFactor.get());
      addConstraint(bulletPhysicsManager, handleHingeConstraint);
   }

   @Override
   public void removeFromBullet()
   {
      super.removeFromBullet();
      doorHingeConstraint.dispose();
      handleHingeConstraint.dispose();
      doorFrameObject.removeFromBullet();
      doorPanelObject.removeFromBullet();
      doorLeverObject.removeFromBullet();
   }

   @Override
   public void updateRenderablesPoses()
   {
      // do nothing
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

//   public static final renderImGuiWidgets()
//   {
//      ImGui.sliderFloat("Low limit", )
//   }
}
