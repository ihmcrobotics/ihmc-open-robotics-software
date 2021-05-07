package us.ihmc.gdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.simulation.environment.object.objects.GDXLabFloorObject;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.simulation.environment.object.objects.GDXPalletObject;
import us.ihmc.gdx.tools.GDXTools;

import java.util.function.Function;

public class GDXEnvironmentObject
{
   protected Model realisticModel;
   private final Pose3D pose = new Pose3D();

   private GDXModelInstance realisticModelInstance;
   private GDXModelInstance collisionModelInstance;
   private Sphere3D boundingSphere;
   private Shape3DBasics collisionGeometryObject;
   private Function<Point3DReadOnly, Boolean> isPointInside;
   private Model collisionMesh;
   private Material originalMaterial;
   private Material highlightedMaterial;
   private final Point3D tempRayOrigin = new Point3D();
   private final Point3D firstSphereIntersection = new Point3D();
   private final Point3D secondSphereIntersection = new Point3D();

   public void create(Model realisticModel)
   {
      this.realisticModel = realisticModel;
      realisticModelInstance = new GDXModelInstance(realisticModel);
   }

   public void create(Model realisticModel,
                      Sphere3D boundingSphere,
                      Shape3DBasics collisionGeometryObject,
                      Function<Point3DReadOnly, Boolean> isPointInside,
                      Model collisionGraphic)
   {
      this.realisticModel = realisticModel;
      this.boundingSphere = boundingSphere;
      this.collisionGeometryObject = collisionGeometryObject;
      this.isPointInside = isPointInside;
      this.collisionMesh = collisionGraphic;
      realisticModelInstance = new GDXModelInstance(realisticModel);
      collisionModelInstance = new GDXModelInstance(collisionGraphic);
      originalMaterial = new Material(realisticModelInstance.materials.get(0));
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

   public void set(Point3DBasics translation)
   {
      GDXTools.toGDX(translation, getRealisticModelInstance().transform);
      GDXTools.toGDX(translation, getCollisionModelInstance().transform);
      getCollisionGeometryObject().getPose().getTranslation().set(translation);
      boundingSphere.getPosition().set(translation);
   }

   public void set(RigidBodyTransform transform)
   {
      GDXTools.toGDX(transform, getRealisticModelInstance().transform);
      GDXTools.toGDX(transform, getCollisionModelInstance().transform);
      getCollisionGeometryObject().getPose().set(transform);
      boundingSphere.getPosition().set(transform.getTranslation());
   }

   public GDXEnvironmentObject duplicate()
   {
      GDXEnvironmentObject duplicate = new GDXEnvironmentObject();
      duplicate.create(realisticModel, boundingSphere, collisionGeometryObject, isPointInside, collisionMesh);
      return duplicate;
   }

   public static GDXEnvironmentObject loadByName(String objectClassName)
   {
      if (objectClassName.equals(GDXMediumCinderBlockRoughed.class.getSimpleName()))
      {
         return new GDXMediumCinderBlockRoughed();
      }
      else if (objectClassName.equals(GDXLabFloorObject.class.getSimpleName()))
      {
         return new GDXLabFloorObject();
      }
      else if (objectClassName.equals(GDXPalletObject.class.getSimpleName()))
      {
         return new GDXPalletObject();
      }
      else
      {
         throw new RuntimeException("There is no object of that name!");
      }
   }
}
