package us.ihmc.gdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.StepCheckIsPointInsideAlgorithm;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * TODO: This class probably needs renaming and setup as layers of interfaces.
 */
public class GDXEnvironmentObject
{
   private static final HashMap<String, AtomicInteger> OBJECT_INDEXES = new HashMap<>();

   private String titleCasedName;
   private final String pascalCasedName;
   private final int objectIndex;
   private final GDXEnvironmentObjectFactory factory;
   private Model realisticModel;
   private GDXModelInstance realisticModelInstance;
   private GDXModelInstance collisionModelInstance;
   private final StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
   private RigidBodyTransform collisionShapeOffset = new RigidBodyTransform();
   private RigidBodyTransform wholeThingOffset = new RigidBodyTransform();
   private Sphere3D boundingSphere = new Sphere3D(1.0);
   private Shape3DBasics collisionGeometryObject;
   private Function<Point3DReadOnly, Boolean> isPointInside;
   private Model collisionMesh;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix4 tempGDXTransform = new Matrix4();
   private final RigidBodyTransform placementTransform = new RigidBodyTransform();
   private final ReferenceFrame placementFrame;
   private final FramePose3D placementFramePose = new FramePose3D();
   private ReferenceFrame realisticModelFrame;
   private ReferenceFrame collisionModelFrame;
   private ReferenceFrame bulletCollisionFrame;
   private ReferenceFrame bulletCollisionSpecificationFrame;
   private final RigidBodyTransform bulletCollisionOffset = new RigidBodyTransform();
   private final RigidBodyTransform bulletCollisionFrameTransformToWorld = new RigidBodyTransform();
   private final FramePose3D bulletPose = new FramePose3D();
   private btCollisionShape btCollisionShape;
   private float mass = 0.0f;
   private btRigidBody btRigidBody;
   private btMultiBody btMultiBody;
   private GDXBulletPhysicsManager bulletPhysicsManager;
   private boolean isSelected = false;
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

   public void copyThisTransformToBulletMultiBody()
   {
      if (btMultiBody != null)
      {
         getThisTransformForCopyToBullet(tempGDXTransform);
         btMultiBody.setBaseWorldTransform(tempGDXTransform);
      }
   }

   public void copyBulletTransformToThis(Matrix4 transformToWorld)
   {
      GDXTools.toEuclid(transformToWorld, bulletCollisionFrameTransformToWorld);
      bulletCollisionFrame.update();
      bulletPose.setToZero(bulletCollisionFrame);
      bulletPose.applyInverseTransform(bulletCollisionOffset);
      bulletPose.changeFrame(ReferenceFrame.getWorldFrame());
      bulletPose.get(tempTransform);
      setTransformToWorld(tempTransform);
   }

   public void getThisTransformForCopyToBullet(Matrix4 transformToWorld)
   {
      bulletCollisionSpecificationFrame.update();
      tempTransform.set(bulletCollisionSpecificationFrame.getTransformToWorldFrame());
      GDXTools.toGDX(tempTransform, transformToWorld);
   }

   public GDXEnvironmentObject(String titleCasedName, GDXEnvironmentObjectFactory factory)
   {
      this.titleCasedName = titleCasedName;
      this.factory = factory;
      pascalCasedName = FormattingTools.titleToKebabCase(titleCasedName);

      AtomicInteger atomicIndex = OBJECT_INDEXES.get(pascalCasedName);
      if (atomicIndex == null)
      {
         atomicIndex = new AtomicInteger(0);
         OBJECT_INDEXES.put(pascalCasedName, atomicIndex);
      }
      objectIndex = atomicIndex.getAndIncrement();
      placementFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "PlacementFrame" + objectIndex,
                                                                                       ReferenceFrame.getWorldFrame(),
                                                                                       placementTransform);
      realisticModelFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "RealisticModelFrame" + objectIndex,
                                                                              placementFrame,
                                                                              wholeThingOffset);
      collisionModelFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "CollisionModelFrame" + objectIndex,
                                                                              realisticModelFrame,
                                                                              collisionShapeOffset);
      bulletCollisionFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "BulletCollisionFrame" + objectIndex,
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              bulletCollisionFrameTransformToWorld);
      bulletCollisionSpecificationFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(pascalCasedName + "BulletCollisionSpecificationFrame" + objectIndex,
                                                                              realisticModelFrame,
                                                                              bulletCollisionOffset);
   }

   public void setRealisticModel(Model realisticModel)
   {
      this.realisticModel = realisticModel;
      realisticModelInstance = new GDXModelInstance(realisticModel);
   }

   public void setCollisionModel(Model collisionGraphic)
   {
      this.collisionMesh = collisionGraphic;
      collisionModelInstance = new GDXModelInstance(collisionGraphic);
   }

   public void setCollisionModel(Consumer<GDXMultiColorMeshBuilder> meshBuilderConsumer)
   {
      Model collisionGraphic = GDXModelPrimitives.buildModel(meshBuilderConsumer, pascalCasedName + "CollisionModel" + getObjectIndex());
      GDXTools.setTransparency(collisionGraphic, 0.4f);
      setCollisionModel(collisionGraphic);
   }

   public void setCollisionGeometryObject(Shape3DBasics collisionGeometryObject)
   {
      this.collisionGeometryObject = collisionGeometryObject;
      isPointInside = collisionGeometryObject::isPointInside;

      if (collisionMesh == null && collisionGeometryObject instanceof Box3D)
      {
         Box3D box3D = (Box3D) collisionGeometryObject;
         setCollisionModel(meshBuilder ->
         {
            Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
            meshBuilder.addBox((float) box3D.getSizeX(), (float) box3D.getSizeY(), (float) box3D.getSizeZ(), color);
            meshBuilder.addMultiLineBox(box3D.getVertices(), 0.01, color); // so we can see it better
         });
      }
   }

   public void setPointIsInsideAlgorithm(Function<Point3DReadOnly, Boolean> isPointInside)
   {
      this.isPointInside = isPointInside;
   }

   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {
      this.bulletPhysicsManager = bulletPhysicsManager;
      if (btCollisionShape != null)
      {
         if (btRigidBody == null)
            btRigidBody = bulletPhysicsManager.addRigidBody(btCollisionShape, mass, getBulletMotionState());
      }
   }

   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager, btMultiBody btMultiBody)
   {
      this.bulletPhysicsManager = bulletPhysicsManager;
      if (btCollisionShape != null)
      {
         if (btMultiBody == null)
         {
            this.btMultiBody = bulletPhysicsManager.addMultiBody(btMultiBody);
         }
      }
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
   }

   /**
    * If we are colliding spheres or boxes, this is overkill. Maybe make this a class that the complicated
    * objects can instantiate? or they can override this method...
    */
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      stepCheckIsPointInsideAlgorithm.setup(boundingSphere.getRadius(), boundingSphere.getPosition());
      return !Double.isNaN(stepCheckIsPointInsideAlgorithm.intersect(pickRay, 100, isPointInside, intersectionToPack, false));
   }

   public void setSelected(boolean selected)
   {
      isSelected = selected;
      if (btRigidBody != null)
      {
         bulletPhysicsManager.setKinematicObject(btRigidBody, selected);
      }
   }

   public void setRawIsSelected(boolean selected)
   {
      isSelected = selected;
   }

   public RigidBodyTransform getCollisionShapeOffset()
   {
      return collisionShapeOffset;
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (realisticModelInstance != null)
         realisticModelInstance.getRenderables(renderables, pool);
   }

   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (collisionModelInstance != null)
         collisionModelInstance.getRenderables(renderables, pool);
   }

   public void setPositionInWorld(Point3DReadOnly positionInWorld)
   {
      placementTransform.getTranslation().set(positionInWorld);
      updateRenderablesPoses();
   }

   public void setPoseInWorld(Pose3D poseInWorld)
   {
      poseInWorld.get(placementTransform);
      updateRenderablesPoses();
   }

   public void setTransformToWorld(Matrix4 transformToWorld)
   {
      GDXTools.toEuclid(transformToWorld, tempTransform);
      setTransformToWorld(tempTransform);
   }

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      placementTransform.set(transformToWorld);
      updateRenderablesPoses();
   }

   public void updateRenderablesPoses()
   {
      placementFrame.update();
      realisticModelFrame.update();
      collisionModelFrame.update();

      placementFramePose.setFromReferenceFrame(realisticModelFrame);
      GDXTools.toGDX(placementFramePose, tempTransform, realisticModelInstance.transform);

      placementFramePose.setFromReferenceFrame(collisionModelFrame);
      GDXTools.toGDX(placementFramePose, tempTransform, collisionModelInstance.transform);
      collisionGeometryObject.getPose().set(placementFramePose);
      boundingSphere.getPosition().set(placementFramePose.getPosition());
   }

   public RigidBodyTransform getObjectTransform()
   {
      return placementTransform;
   }

   public GDXEnvironmentObject duplicate()
   {
      return factory.getSupplier().get();
   }

   public String getTitleCasedName()
   {
      return titleCasedName;
   }

   public String getPascalCasedName()
   {
      return pascalCasedName;
   }

   public int getObjectIndex()
   {
      return objectIndex;
   }

   public GDXEnvironmentObjectFactory getFactory()
   {
      return factory;
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

   public RigidBodyTransform getWholeThingOffset()
   {
      return wholeThingOffset;
   }

   public Sphere3D getBoundingSphere()
   {
      return boundingSphere;
   }

   public boolean getIsSelected()
   {
      return isSelected;
   }
}
