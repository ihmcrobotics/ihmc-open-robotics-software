package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBoxShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.simulation.environment.object.objects.door.GDXDoorFrameObject;
import us.ihmc.gdx.simulation.environment.object.objects.door.GDXDoorLeverHandleObject;
import us.ihmc.gdx.simulation.environment.object.objects.door.GDXDoorPanelObject;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;

import java.util.ArrayList;

public class GDXMultiBodySnakeObject extends GDXEnvironmentObject
{
   public static final String NAME = "Multi Body Snake";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXMultiBodySnakeObject.class);
   private final Matrix4 tempGDXTransform = new Matrix4();
   private btMultiBody multiBody;
//   private ModelInstance baseModelInstance;
   private final ArrayList<ModelInstance> linkModelInstances = new ArrayList<>();
   private int numberOfLinks;

   public GDXMultiBodySnakeObject()
   {
      super(NAME, FACTORY);
      double length = 0.05;
      double width = 0.1;
      double height = 0.37;

      Model realisticModel = GDXModelPrimitives.createBox((float) height * 2.0f, (float) width * 2.0f, (float) length * 2.0f, Color.BLUE).model;
      setRealisticModel(realisticModel);

      setMass(1.0f);
      Box3D collisionBox = new Box3D(length * 2.0, width * 2.0, height * 2.0);
      setCollisionGeometryObject(collisionBox);
      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);
   }

   @Override
   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {
      numberOfLinks = 5;
      float mass = 1.0f;
      boolean fixedBase = false;
      boolean canSleep = false;

      Vector3 linkHalfExtents = new Vector3(0.05f, 0.1f, 0.37f);
      Vector3 baseInertiaDiagonal = new Vector3(0.0f, 0.0f, 0.0f);

      btBoxShape baseBox = new btBoxShape(linkHalfExtents);
      baseBox.calculateLocalInertia(mass, baseInertiaDiagonal);

      multiBody = new btMultiBody(numberOfLinks, mass, baseInertiaDiagonal, fixedBase, canSleep);
      multiBody.setBasePos(new Vector3());
      multiBody.setWorldToBaseRot(new Quaternion());

      btMultiBodyLinkCollider baseCollider = new btMultiBodyLinkCollider(multiBody, -1);
      baseCollider.setCollisionShape(baseBox);
      Matrix4 worldTransform = new Matrix4();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toGDX(rigidBodyTransform, worldTransform);
      baseCollider.setWorldTransform(worldTransform);
      bulletPhysicsManager.getMultiBodyDynamicsWorld().addCollisionObject(baseCollider);
      multiBody.setBaseCollider(baseCollider);

//      baseModelInstance = GDXModelPrimitives.createBox(linkHalfExtents.x * 2.0f, linkHalfExtents.y * 2.0f, linkHalfExtents.z * 2.0f, Color.BLUE);

      for (int i = 0; i < numberOfLinks; i++)
      {
         int linkIndex = i;
         mass = 1.0f;
         int parentIndex = linkIndex - 1;
         Quaternion rotationFromParent = new Quaternion();
         Vector3 offsetOfPivotFromParentCenterOfMass = new Vector3(0.0f, 0.0f, -linkHalfExtents.z * 2.0f);
         Vector3 offsetOfCenterOfMassFromPivot = new Vector3(0.0f, 0.0f, -linkHalfExtents.z);
         boolean disableParentCollision = false;
         multiBody.setupSpherical(linkIndex,
                                  mass,
                                  baseInertiaDiagonal,
                                  parentIndex,
                                  rotationFromParent,
                                  offsetOfPivotFromParentCenterOfMass,
                                  offsetOfCenterOfMassFromPivot,
                                  disableParentCollision);

         btMultiBodyLinkCollider linkCollider = new btMultiBodyLinkCollider(multiBody, linkIndex);
         btBoxShape linkBox = new btBoxShape(linkHalfExtents);
         linkCollider.setCollisionShape(linkBox);
         linkCollider.setWorldTransform(worldTransform);
         bulletPhysicsManager.getMultiBodyDynamicsWorld().addCollisionObject(linkCollider);
         multiBody.getLink(linkIndex).setCollider(linkCollider);

         linkModelInstances.add(GDXModelPrimitives.createBox(linkHalfExtents.x * 2.0f, linkHalfExtents.y * 2.0f, linkHalfExtents.z * 2.0f, Color.RED));
      }

      multiBody.finalizeMultiDof();
      bulletPhysicsManager.addMultiBody(multiBody);
   }

   @Override
   public void copyBulletTransformToThisMultiBody()
   {
//      baseModelInstance.transform.set(multiBody.getBaseCollider().getWorldTransform());
      setTransformToWorld(multiBody.getBaseCollider().getWorldTransform());
      for (int i = 0; i < numberOfLinks; i++)
      {
         linkModelInstances.get(i).transform.set(multiBody.getLink(i).getCollider().getWorldTransform());
      }
   }

   @Override
   public void copyThisTransformToBulletMultiBody()
   {
      getThisTransformForCopyToBullet(tempGDXTransform);
      multiBody.setBaseWorldTransform(tempGDXTransform);
   }

//   @Override
//   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
//   {
//      return doorFrameObject.intersect(pickRay, intersectionToPack);
//   }
//
//   @Override
//   public void setSelected(boolean selected)
//   {
//      doorFrameObject.setSelected(selected);
//   }
//
//   @Override
//   public void setPositionInWorld(Point3DReadOnly positionInWorld)
//   {
//      doorFrameObject.setPositionInWorld(positionInWorld);
//   }
//
//   @Override
//   public void setPoseInWorld(Pose3D poseInWorld)
//   {
//      doorFrameObject.setPoseInWorld(poseInWorld);
//   }
//
//   @Override
//   public void setTransformToWorld(RigidBodyTransform transformToWorld)
//   {
//      doorFrameObject.setTransformToWorld(transformToWorld);
//   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRealRenderables(renderables, pool);
      for (ModelInstance linkModelInstance : linkModelInstances)
      {
         linkModelInstance.getRenderables(renderables, pool);
      }
   }

//   @Override
//   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
//   {
//      doorFrameObject.getCollisionMeshRenderables(renderables, pool);
//      doorPanelObject.getCollisionMeshRenderables(renderables, pool);
//      doorLeverObject.getCollisionMeshRenderables(renderables, pool);
//   }

//   @Override
//   public RigidBodyTransform getObjectTransform()
//   {
//      return doorFrameObject.getObjectTransform();
//   }
//
//   @Override
//   public boolean getIsSelected()
//   {
//      return doorFrameObject.getIsSelected();
//   }
}
