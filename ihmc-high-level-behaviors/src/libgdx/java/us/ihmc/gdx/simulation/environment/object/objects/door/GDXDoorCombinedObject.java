package us.ihmc.gdx.simulation.environment.object.objects.door;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btHingeConstraint;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;

public class GDXDoorCombinedObject extends GDXEnvironmentObject
{
   public static final String NAME = "Door Combined";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXDoorCombinedObject.class);
   private final GDXDoorPanelObject doorPanelObject;
   private final GDXDoorLeverHandleObject doorLeverObject;

   public GDXDoorCombinedObject()
   {
      super(NAME, FACTORY);

      doorPanelObject = new GDXDoorPanelObject();
      doorLeverObject = new GDXDoorLeverHandleObject();
   }

   @Override
   protected void updateRenderablesPoses()
   {
      // do nothing actually
   }

   @Override
   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {
      doorPanelObject.addToBullet(bulletPhysicsManager);
      doorLeverObject.addToBullet(bulletPhysicsManager);

      Vector3 pivotInPanel = new Vector3(0.0f, 0.85f, 0.9f);
      Vector3 pivotInLever = new Vector3(0.0f, 0.0f, 0.0f);
      Vector3 axisInPanel = new Vector3(1.0f, 0.0f, 0.0f);
      Vector3 axisInLever = new Vector3(1.0f, 0.0f, 0.0f);
      boolean useReferenceFrameA = true;
      btHingeConstraint btHingeConstraint = new btHingeConstraint(doorPanelObject.getBtRigidBody(),
                                                                  doorLeverObject.getBtRigidBody(),
                                                                  pivotInPanel,
                                                                  pivotInLever,
                                                                  axisInPanel,
                                                                  axisInLever,
                                                                  useReferenceFrameA);
      // these limits from 0.0 to 1.0?
      float lowLimit = 0.5f;
      float highLimit = 0.5f;
//      btHingeConstraint.setLimit(lowLimit, highLimit);
//      float relaxationFactor;
//      float biasFactor;
//      float softness;
//      btHingeConstraint.setLimit(lowLimit, highLimit, softness, biasFactor, relaxationFactor);

      bulletPhysicsManager.getDiscreteDynamicsWorld().addConstraint(btHingeConstraint);
   }

   @Override
   public void removeFromBullet()
   {
      doorPanelObject.removeFromBullet();
      doorLeverObject.removeFromBullet();
   }

   @Override
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      return doorPanelObject.intersect(pickRay, intersectionToPack);
   }

   @Override
   public void setSelected(boolean selected)
   {
      doorPanelObject.setSelected(selected);
   }

   @Override
   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      doorPanelObject.setPositionInWorld(positionInWorld);
   }

   @Override
   public void setPoseInWorld(Pose3D poseInWorld)
   {
      doorPanelObject.setPoseInWorld(poseInWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      doorPanelObject.setTransformToWorld(transformToWorld);
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      doorPanelObject.getRealRenderables(renderables, pool);
      doorLeverObject.getRealRenderables(renderables, pool);
   }

   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      doorPanelObject.getCollisionMeshRenderables(renderables, pool);
      doorLeverObject.getCollisionMeshRenderables(renderables, pool);
   }

   @Override
   public RigidBodyTransformReadOnly getObjectTransform()
   {
      return doorPanelObject.getObjectTransform();
   }
}
