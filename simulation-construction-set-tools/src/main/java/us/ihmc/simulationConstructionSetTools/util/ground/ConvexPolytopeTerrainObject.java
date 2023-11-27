package us.ihmc.simulationConstructionSetTools.util.ground;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.physics.CollidableVisualizer;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class ConvexPolytopeTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final BoundingBox3D boundingBox;
   private final ConvexPolytope3D convexPolytope;
   private final Graphics3DObject linkGraphics;

   private static final double EPSILON = 1.0e-12;

   public ConvexPolytopeTerrainObject(ConvexPolytope3D convexPolytope)
   {
      this(convexPolytope, YoAppearance.StoneTexture());
   }

   public ConvexPolytopeTerrainObject(ConvexPolytope3D convexPolytope, AppearanceDefinition appearance)
   {
      this.convexPolytope = new ConvexPolytope3D(convexPolytope);
      this.boundingBox = new BoundingBox3D(convexPolytope.getBoundingBox().getMinPoint(), convexPolytope.getBoundingBox().getMaxPoint());
      this.linkGraphics = new Graphics3DObject();
      this.linkGraphics.addMeshData(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), appearance);
   }

   public IntersectionResult intersectionWithVerticalLine(double x, double y)
   {
      Point3D point = new Point3D(x, y, 0);

      double highest = Double.NEGATIVE_INFINITY;
      double lowest = Double.POSITIVE_INFINITY;

      Face3DReadOnly highestFace = null;
      Face3DReadOnly lowestFace = null;

      // Find the highest and lowest points on the faces of the ConvexPolytope that
      // the point passes through along the Z-Axis.
      // If may go through multiple points or no point.
      for (Face3DReadOnly face : convexPolytope.getFaces())
      {
         Point3D pointIntersectionLineAndFace = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(face.getCentroid(), face.getNormal(), point, Axis3D.Z);
         if (pointIntersectionLineAndFace != null)
         {
            if (face.distance(pointIntersectionLineAndFace) <= EPSILON)
            {
               if (pointIntersectionLineAndFace.getZ() > highest)
               {
                  highest = pointIntersectionLineAndFace.getZ();
                  highestFace = face;
               }
               if (pointIntersectionLineAndFace.getZ() < lowest)
               {
                  lowest = pointIntersectionLineAndFace.getZ();
                  lowestFace = face;
               }
            }
         }
      }

      return new IntersectionResult(new Point3D(x, y, highest), highestFace, new Point3D(x, y, lowest), lowestFace);

   }

   static class IntersectionResult
   {
      private final Point3D highestIntersection;
      private final Face3DReadOnly highestFace;

      private final Point3D lowestIntersection;
      private final Face3DReadOnly lowestFace;

      public IntersectionResult(Point3D highestIntersection, Face3DReadOnly highestFace, Point3D lowestIntersection, Face3DReadOnly lowestFace)
      {
         this.highestIntersection = highestIntersection;
         this.highestFace = highestFace;
         this.lowestIntersection = lowestIntersection;
         this.lowestFace = lowestFace;
      }

      public boolean isHighestPointValid()
      {
         return highestIntersection.getZ() != Double.NEGATIVE_INFINITY;
      }

      public Point3D getHighestIntersection()
      {
         return highestIntersection;
      }

      public Face3DReadOnly getHighestFace()
      {
         return highestFace;
      }

      public Point3D getLowestIntersection()
      {
         return lowestIntersection;
      }

      public Face3DReadOnly getLowestFace()
      {
         return lowestFace;
      }
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      IntersectionResult result = intersectionWithVerticalLine(x, y);
      // Determine the height of the point on the zAxis of the ConvexPolytope -
      // returns boundingBox.getMinZ() if it does
      // not pass through the ConvexPolytope or the point is completely below the
      // ConvexPolytope
      if (result.isHighestPointValid() && z >= result.lowestIntersection.getZ())
         return result.highestIntersection.getZ();

      return Double.NEGATIVE_INFINITY;
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      IntersectionResult result = intersectionWithVerticalLine(x, y);

      double heightAt;

      if (result.isHighestPointValid() && z >= result.lowestIntersection.getZ())
         heightAt = result.highestIntersection.getZ();
      else
         heightAt = Double.NEGATIVE_INFINITY;

      if (intersectionToPack != null)
      {
         intersectionToPack.setX(x);
         intersectionToPack.setY(y);
         intersectionToPack.setZ(heightAt);
      }

      if (normalToPack != null && heightAt > Double.NEGATIVE_INFINITY)
      {
         normalToPack.set(result.highestFace.getNormal());
      }

      return (z < heightAt);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      IntersectionResult result = intersectionWithVerticalLine(x, y);

      double heightAt;

      if (result.isHighestPointValid() && z >= result.lowestIntersection.getZ())
         heightAt = result.highestIntersection.getZ();
      else
         heightAt = Double.NEGATIVE_INFINITY;
      if (normalToPack != null && heightAt > Double.NEGATIVE_INFINITY)
      {
         normalToPack.set(result.highestFace.getNormal());
      }

      return heightAt;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if (boundingBox == null)
         return false;

      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   @Override
   public List<? extends Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return Collections.singletonList(convexPolytope);
   }

}
