package us.ihmc.gdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
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
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXTools;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;

public class GDXEnvironmentObject
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   private String name;
   private final GDXEnvironmentObjectFactory factory;
   private Model realisticModel;
   private GDXModelInstance realisticModelInstance;
   private GDXModelInstance collisionModelInstance;
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
   private final ReferenceFrame placementFrame
         = ReferenceFrameTools.constructFrameWithChangingTransformToParent("placementFrame" + INDEX.getAndIncrement(),
                                                                           ReferenceFrame.getWorldFrame(),
                                                                           placementTransform);
   private final FramePose3D placementFramePose = new FramePose3D();
   private ReferenceFrame realisticModelFrame;
   private ReferenceFrame collisionModelFrame;

   public GDXEnvironmentObject(String name, GDXEnvironmentObjectFactory factory)
   {
      this.name = name;
      this.factory = factory;
   }

   public void create(Model realisticModel)
   {
      this.realisticModel = realisticModel;
      realisticModelInstance = new GDXModelInstance(realisticModel);
   }

   public void create(Model realisticModel,
                      RigidBodyTransform collisionShapeOffset,
                      RigidBodyTransform wholeThingOffset,
                      Sphere3D boundingSphere,
                      Shape3DBasics collisionGeometryObject,
                      Function<Point3DReadOnly, Boolean> isPointInside,
                      Model collisionGraphic)
   {
      this.realisticModel = realisticModel;
      this.collisionShapeOffset = collisionShapeOffset;
      this.wholeThingOffset = wholeThingOffset;
      this.boundingSphere = boundingSphere;
      this.collisionGeometryObject = collisionGeometryObject;
      this.isPointInside = isPointInside;
      this.collisionMesh = collisionGraphic;
      realisticModelInstance = new GDXModelInstance(realisticModel);
      collisionModelInstance = new GDXModelInstance(collisionGraphic);
      originalMaterial = new Material(realisticModelInstance.materials.get(0));
      realisticModelFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("realisticModelFrame" + INDEX.getAndIncrement(),
                                                                                                placementFrame,
                                                                                                wholeThingOffset);
      collisionModelFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("collisionModelFrame" + INDEX.getAndIncrement(),
                                                                                                realisticModelFrame,
                                                                                                collisionShapeOffset);
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

   public void set(Point3DReadOnly translation)
   {
      placementTransform.getTranslation().set(translation);
      updateRenderablesPoses();
   }

   public void set(Pose3D pose)
   {
      pose.get(placementTransform);
      updateRenderablesPoses();
   }

   public void set(RigidBodyTransform transform)
   {
      placementTransform.set(transform);
      updateRenderablesPoses();
   }

   protected void updateRenderablesPoses()
   {
      placementFrame.update();

      placementFramePose.setFromReferenceFrame(realisticModelFrame);
      GDXTools.toGDX(placementFramePose, tempTransform, getRealisticModelInstance().transform);

      placementFramePose.setFromReferenceFrame(collisionModelFrame);
      GDXTools.toGDX(placementFramePose, tempTransform, getCollisionModelInstance().transform);
      getCollisionGeometryObject().getPose().set(placementFramePose);
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

   public String getName()
   {
      return name;
   }

   public GDXEnvironmentObjectFactory getFactory()
   {
      return factory;
   }
}
