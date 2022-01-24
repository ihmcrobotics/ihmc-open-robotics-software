package us.ihmc.gdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.simulation.environment.GDXBulletPhysicsManager;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXTools;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;

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
   private double mass;
   private RigidBodyTransform collisionShapeOffset;
   private RigidBodyTransform wholeThingOffset;
   private Sphere3D boundingSphere;
   private Shape3DBasics collisionGeometryObject;
   private Function<Point3DReadOnly, Boolean> isPointInside;
   private Model collisionMesh;
   private Material originalMaterial;
   private Material highlightedMaterial;
   private final Point3D tempRayOrigin = new Point3D();
   private final Point3D firstSphereIntersection = new Point3D();
   private final Point3D secondSphereIntersection = new Point3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBodyTransform placementTransform = new RigidBodyTransform();
   private final ReferenceFrame placementFrame;
   private final FramePose3D placementFramePose = new FramePose3D();
   private ReferenceFrame realisticModelFrame;
   private ReferenceFrame collisionModelFrame;
   private Matrix4 bulletTransform = new Matrix4();
   private btMotionState bulletMotionState = new btMotionState()
   {
      @Override
      public void setWorldTransform(Matrix4 transformToWorld)
      {
         setTransformToWorld(transformToWorld);
      }

      @Override
      public void getWorldTransform(Matrix4 transformToWorld)
      {
         GDXTools.toGDX(collisionModelFrame.getTransformToWorldFrame(), transformToWorld);
      }
   };

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
   }

   public void create(Model realisticModel,
                      Model collisionGraphic,
                      RigidBodyTransform collisionShapeOffset,
                      RigidBodyTransform wholeThingOffset,
                      Sphere3D boundingSphere,
                      Shape3DBasics collisionGeometryObject,
                      Function<Point3DReadOnly, Boolean> isPointInside,
                      double mass)
   {
      this.realisticModel = realisticModel;
      this.collisionShapeOffset = collisionShapeOffset;
      this.wholeThingOffset = wholeThingOffset;
      this.boundingSphere = boundingSphere;
      this.collisionGeometryObject = collisionGeometryObject;
      this.isPointInside = isPointInside;
      this.collisionMesh = collisionGraphic;
      this.mass = mass;
      realisticModelInstance = new GDXModelInstance(realisticModel);
      collisionModelInstance = new GDXModelInstance(collisionGraphic);
      originalMaterial = new Material(realisticModelInstance.materials.get(0));
      realisticModelFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(pascalCasedName + "RealisticModelFrame" + objectIndex,
                                                                                              placementFrame,
                                                                                              wholeThingOffset);
      collisionModelFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(pascalCasedName + "CollisionModelFrame" + objectIndex,
                                                                                              realisticModelFrame,
                                                                                              collisionShapeOffset);
   }

   public void addToBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {

   }

   public void removeFromBullet(GDXBulletPhysicsManager bulletPhysicsManager)
   {

   }

   /**
    * If we are colliding spheres or boxes, this is overkill. Maybe make this a class that the complicated
    * objects can instantiate? or they can override this method...
    */
   public boolean intersect(Line3DReadOnly pickRay, Point3D intersectionToPack)
   {
      tempRayOrigin.setX(pickRay.getPoint().getX() - boundingSphere.getPosition().getX());
      tempRayOrigin.setY(pickRay.getPoint().getY() - boundingSphere.getPosition().getY());
      tempRayOrigin.setZ(pickRay.getPoint().getZ() - boundingSphere.getPosition().getZ());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             tempRayOrigin,
                                                                                             pickRay.getDirection(),
                                                                                             firstSphereIntersection,
                                                                                             secondSphereIntersection);
      if (numberOfIntersections == 2)
      {
         firstSphereIntersection.add(boundingSphere.getPosition());
         secondSphereIntersection.add(boundingSphere.getPosition());
         for (int i = 0; i < 100; i++)
         {
            intersectionToPack.interpolate(firstSphereIntersection, secondSphereIntersection, i / 100.0);
            if (isPointInside.apply(intersectionToPack))
            {
               return true;
            }
         }
      }

      return false;
   }

   public void setHighlighted(boolean highlighted)
   {
      if (highlighted)
      {
         if (highlightedMaterial == null)
         {
            highlightedMaterial = new Material();
            highlightedMaterial.set(ColorAttribute.createDiffuse(Color.ORANGE));
         }

         realisticModelInstance.materials.get(0).set(highlightedMaterial);
      }
      else
      {
         realisticModelInstance.materials.get(0).set(originalMaterial);
      }
   }

   public RigidBodyTransform getCollisionShapeOffset()
   {
      return collisionShapeOffset;
   }

   public GDXModelInstance getRealisticModelInstance()
   {
      return realisticModelInstance;
   }

   public GDXModelInstance getCollisionModelInstance()
   {
      return collisionModelInstance;
   }

   public Shape3DBasics getCollisionGeometryObject()
   {
      return collisionGeometryObject;
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

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      placementTransform.set(transformToWorld);
      updateRenderablesPoses();
   }

   public void setTransformToWorld(Matrix4 transformToWorld)
   {
      GDXTools.toEuclid(transformToWorld, placementTransform);
      updateRenderablesPoses();
   }

   protected void updateRenderablesPoses()
   {
      placementFrame.update();

      placementFramePose.setFromReferenceFrame(realisticModelFrame);
      GDXTools.toGDX(placementFramePose, tempTransform, getRealisticModelInstance().transform);

      placementFramePose.setFromReferenceFrame(collisionModelFrame);
      GDXTools.toGDX(placementFramePose, tempTransform, getCollisionModelInstance().transform);
      collisionGeometryObject.getPose().set(placementFramePose);
      boundingSphere.getPosition().set(placementFramePose.getPosition());
   }

   public RigidBodyTransformReadOnly getObjectTransform()
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
}
