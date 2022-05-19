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
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.simulation.bullet.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;

import java.util.ArrayList;

public class GDXMultiBodySnakeObject extends GDXEnvironmentObject
{
   public static final String NAME = "Multi Body Snake";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXMultiBodySnakeObject.class);
   private final Matrix4 tempGDXTransform = new Matrix4();
   private btMultiBody multiBody;
   private final ArrayList<ModelInstance> linkModelInstances = new ArrayList<>();
   private int numberOfLinks;
   private RigidBodyTransform tempTransform = new RigidBodyTransform();
   private Vector3 linkHalfExtents = new Vector3(0.05f, 0.1f, 0.37f);
   private btMultiBodyLinkCollider baseCollider; // must store these or JNI wrapper will allocate on get and crash
   private final ArrayList<btMultiBodyLinkCollider> linkColliders = new ArrayList<>(); // must store these or JNI wrapper will allocate on get and crash

   public GDXMultiBodySnakeObject()
   {
      super(NAME, FACTORY);

      Model realisticModel = GDXModelBuilder.createBox(linkHalfExtents.x * 2.0f, linkHalfExtents.y * 2.0f, linkHalfExtents.z * 2.0f, Color.BLUE).model;
      setRealisticModel(realisticModel);

      setMass(1.0f);
      Box3D collisionBox = new Box3D(linkHalfExtents.x * 2.0, linkHalfExtents.y * 2.0, linkHalfExtents.z * 2.0);
      setCollisionGeometryObject(collisionBox);
      getBoundingSphere().setRadius(collisionBox.getSize().length() / 2.0);
   }

   @Override
   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {
      numberOfLinks = 5;
      boolean fixedBase = false;
      boolean canSleep = false;

      Vector3 baseInertiaDiagonal = new Vector3(0.0f, 0.0f, 0.0f);

      btBoxShape baseBox = new btBoxShape(linkHalfExtents);
      baseBox.calculateLocalInertia(getMass(), baseInertiaDiagonal);

      multiBody = new btMultiBody(numberOfLinks, getMass(), baseInertiaDiagonal, fixedBase, canSleep);
      multiBody.setBasePos(new Vector3());
      multiBody.setWorldToBaseRot(new Quaternion());
      multiBody.setHasSelfCollision(true);
      multiBody.setUseGyroTerm(true);
      multiBody.setLinearDamping(0.1f);
      multiBody.setAngularDamping(0.9f);

      baseCollider = new btMultiBodyLinkCollider(multiBody, -1);
      baseCollider.setCollisionShape(baseBox);
      Matrix4 worldTransform = new Matrix4();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toGDX(rigidBodyTransform, worldTransform);
      baseCollider.setWorldTransform(worldTransform);
      baseCollider.setFriction(1.0f);
      addMultiBodyCollisionShape(bulletPhysicsManager, baseCollider);
      multiBody.setBaseCollider(baseCollider);

      for (int i = 0; i < numberOfLinks; i++)
      {
         int linkIndex = i;
         int parentIndex = linkIndex - 1;
         Quaternion rotationFromParent = new Quaternion();
         Vector3 offsetOfPivotFromParentCenterOfMass = new Vector3(0.0f, 0.0f, -linkHalfExtents.z);
         Vector3 offsetOfCenterOfMassFromPivot = new Vector3(0.0f, 0.0f, -linkHalfExtents.z);
         boolean disableParentCollision = true;
         multiBody.setupSpherical(linkIndex,
                                  getMass(),
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
         linkCollider.setFriction(1.0f);
         addMultiBodyCollisionShape(bulletPhysicsManager, linkCollider);
         multiBody.getLink(linkIndex).setCollider(linkCollider);
         linkColliders.add(linkCollider);

         linkModelInstances.add(GDXModelBuilder.createBox(linkHalfExtents.x * 2.0f, linkHalfExtents.y * 2.0f, linkHalfExtents.z * 2.0f, Color.RED));
      }

      multiBody.finalizeMultiDof();
      addBtMultiBody(bulletPhysicsManager, multiBody);
   }

   @Override
   public void copyBulletTransformToThisMultiBody()
   {
      setTransformToWorld(baseCollider.getWorldTransform());
      for (int i = 0; i < numberOfLinks; i++)
      {
         linkModelInstances.get(i).transform.set(linkColliders.get(i).getWorldTransform());
      }
   }

   @Override
   public void copyThisTransformToBulletMultiBodyParentOnly()
   {
      getThisTransformForCopyToBullet(tempGDXTransform);
      multiBody.setBaseWorldTransform(tempGDXTransform);
      baseCollider.setWorldTransform(tempGDXTransform);
   }

   @Override
   public void copyThisTransformToBulletMultiBody()
   {
      copyThisTransformToBulletMultiBodyParentOnly();

      for (int i = 0; i < numberOfLinks; i++)
      {
         GDXTools.toEuclid(tempGDXTransform, tempTransform);
         tempTransform.appendTranslation(0.0, 0.0, -linkHalfExtents.z * 2.0f);
         GDXTools.toGDX(tempTransform, tempGDXTransform);
         linkColliders.get(i).setWorldTransform(tempGDXTransform);
         linkModelInstances.get(i).transform.set(tempGDXTransform);
      }
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRealRenderables(renderables, pool);
      for (ModelInstance linkModelInstance : linkModelInstances)
      {
         linkModelInstance.getRenderables(renderables, pool);
      }
   }
}
