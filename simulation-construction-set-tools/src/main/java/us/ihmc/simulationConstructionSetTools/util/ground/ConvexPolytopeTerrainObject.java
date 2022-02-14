package us.ihmc.simulationConstructionSetTools.util.ground;

import java.util.ArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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

   public ConvexPolytopeTerrainObject(Vector3D normal, ConvexPolytope3D convexPolytope)
   {
      this(normal, convexPolytope, YoAppearance.StoneTexture());
   }

   public ConvexPolytopeTerrainObject(Vector3D normal, ConvexPolytope3D convexPolytope, AppearanceDefinition appearance)
   {
      if (normal.getZ() <= 0.0)
         throw new RuntimeException("Top surface normal must have a positive z-value. Normal.z = " + normal.getZ());

      this.convexPolytope = new ConvexPolytope3D(convexPolytope);
      this.boundingBox = new BoundingBox3D(convexPolytope.getBoundingBox().getMinPoint(), convexPolytope.getBoundingBox().getMaxPoint());
      this.linkGraphics = new Graphics3DObject();
      this.linkGraphics.addMeshData(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), appearance);
   }

   public double heightAt(double x, double y, double z)
   {
      Point3D point = new Point3D(x, y, z);
      double highest = Double.NEGATIVE_INFINITY;
      double lowest = Double.POSITIVE_INFINITY;

      //Find the highest and lowest points on the faces of the ConvexPolytope that the point passes through along the Z-Axis.
      //If may go through multiple points or no point.
      for (Face3DReadOnly face : convexPolytope.getFaces())
      {
         Point3D pointIntersectionLineAndFace = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(face.getCentroid(), face.getNormal(), point, Axis3D.Z);

         if (pointIntersectionLineAndFace != null)
         {
            if (face.distance(pointIntersectionLineAndFace) <= EPSILON)
            {
               highest = Math.max(highest, pointIntersectionLineAndFace.getZ());
               lowest = Math.min(lowest, pointIntersectionLineAndFace.getZ());
            }
         }
      }

      //Determine the height of the point on the zAxis of the ConvexPolytope - returns boundingBox.getMinZ() if it does
      //not pass through the ConvexPolytope or the point is completely below the ConvexPolytope
      if (highest != Double.NEGATIVE_INFINITY && z >= lowest)
         return highest;

      return Double.NEGATIVE_INFINITY;
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      intersectionToPack.setX(x);
      intersectionToPack.setY(y);
      intersectionToPack.setZ(heightAt(x, y, z));
      
      if (intersectionToPack.getZ() > Double.NEGATIVE_INFINITY)
         normalToPack.set(convexPolytope.getClosestFace(intersectionToPack).getNormal());

      return (z < intersectionToPack.getZ());
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      Point3D intersectionToIgnore = new Point3D();
      
      checkIfInside(x, y, z, intersectionToIgnore, normalToPack);
      
      return heightAt(x, y, z);
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

}
