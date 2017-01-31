package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.LineSegment3d;
import us.ihmc.robotics.geometry.shapes.Plane3d;


public class RotatableConvexPolygonTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final BoundingBox3d boundingBox;
   private final ConvexPolygon2d convexPolygon;
   private final Plane3d topPlane;
   private final List<Plane3d> sidePlanes = new ArrayList<Plane3d>();

   private final Graphics3DObject linkGraphics;
   private final AppearanceDefinition appearance;

   private static final double EPSILON = Double.MIN_VALUE;
   private static final boolean VISUALIZE_SURFACE_NORMALS = false;

   public RotatableConvexPolygonTerrainObject(Vector3d normal, ConvexPolygon2d convexPolygon, double centroidHeight)
   {
      this(normal, convexPolygon, centroidHeight, YoAppearance.StoneTexture());
   }

   public RotatableConvexPolygonTerrainObject(Vector3d normal, ConvexPolygon2d convexPolygon, double centroidHeight, AppearanceDefinition appearance)
   {
      if (normal.getZ() <= 0.0)
         throw new RuntimeException("Top surface normal must have a positive z-value. Normal.z = " + normal.getZ());
      this.convexPolygon = new ConvexPolygon2d(convexPolygon);
      Point3d centroid = new Point3d(convexPolygon.getCentroid().getX(), convexPolygon.getCentroid().getY(), centroidHeight);
      this.topPlane = new Plane3d(centroid, normal);

      BoundingBox2d polygonBoundingBox = convexPolygon.getBoundingBoxCopy();
      double highest = Double.NEGATIVE_INFINITY;
      double lowest = 0.0;

      double height;
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = convexPolygon.getVertex(i);
         height = heightAt(vertex.getX(), vertex.getY(), 0.0);
         if (height < lowest)
            lowest = height;
         if (height > highest)
            highest = height;
      }

      Point2d minPoint = polygonBoundingBox.getMinPoint();
      Point2d maxPoint = polygonBoundingBox.getMaxPoint();
      double[] minPoint3d = {minPoint.getX(), minPoint.getY(), lowest};
      double[] maxPoint3d = {maxPoint.getX(), maxPoint.getY(), highest};

      this.boundingBox = new BoundingBox3d(minPoint3d, maxPoint3d);

      initSidePlanes();

      this.linkGraphics = new Graphics3DObject();
      this.appearance = appearance;
      addLinkGraphics(convexPolygon, boundingBox);
   }

   private void addLinkGraphics(ConvexPolygon2d convexPolygon, BoundingBox3d boundingBox)
   {
      double lowValue = boundingBox.getZMin();

      Point2d firstPoint, secondPoint;
      Point3d[] topPlanePoints = new Point3d[convexPolygon.getNumberOfVertices()];
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point3d[] sidePlanePoints = new Point3d[4];

         firstPoint = convexPolygon.getVertexCCW(i);
         secondPoint = convexPolygon.getNextVertexCCW(i);

         sidePlanePoints[0] = new Point3d(firstPoint.getX(), firstPoint.getY(), lowValue);
         sidePlanePoints[1] = new Point3d(secondPoint.getX(), secondPoint.getY(), lowValue);
         sidePlanePoints[2] = new Point3d(secondPoint.getX(), secondPoint.getY(), heightAt(secondPoint.getX(), secondPoint.getY(), 0.0));
         sidePlanePoints[3] = new Point3d(firstPoint.getX(), firstPoint.getY(), heightAt(firstPoint.getX(), firstPoint.getY(), 0.0));

         this.linkGraphics.addPolygon(sidePlanePoints, appearance);

         topPlanePoints[i] = new Point3d(sidePlanePoints[3]);
      }

      this.linkGraphics.addPolygon(topPlanePoints, appearance);

      if(VISUALIZE_SURFACE_NORMALS)
      {
         visualizeNormalVector(topPlane);

         for (Plane3d sidePlane : sidePlanes)
         {
            visualizeNormalVector(sidePlane);
         }
      }
   }

   private void visualizeNormalVector(Plane3d plane)
   {
      this.linkGraphics.identity();
      linkGraphics.translate(new Vector3d(plane.getPointCopy()));
      linkGraphics.addSphere(0.005, YoAppearance.Black());

      Vector3d normalCopy = plane.getNormalCopy();
      normalCopy.scale(0.01);
      linkGraphics.translate(normalCopy);
      linkGraphics.addSphere(0.005, YoAppearance.Blue());
   }

   private void initSidePlanes()
   {
      Vector2d normal2d = new Vector2d();

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         ConvexPolygon2dCalculator.getEdgeNormal(i, normal2d, convexPolygon);
         Vector3d normal = new Vector3d(normal2d.getX(), normal2d.getY(), 0.0);

         int secondIndex = (i + 1 < convexPolygon.getNumberOfVertices()) ? i + 1 : 0;
         int[] indices = {i, secondIndex};

         Point3d centerPoint = new Point3d();
         Point3d checkingPoint = new Point3d();

         for (int j : indices)
         {
            Point2d vertex = convexPolygon.getVertex(j);
            checkingPoint.set(vertex.getX(), vertex.getY(), boundingBox.getZMin());
            checkingPoint.scale(0.25);
            centerPoint.add(checkingPoint);

            checkingPoint.set(vertex.getX(), vertex.getY(), heightAt(vertex.getX(), vertex.getY(), 0.0));
            checkingPoint.scale(0.25);
            centerPoint.add(checkingPoint);
         }

         Plane3d sidePlane = new Plane3d(centerPoint, normal);
         sidePlanes.add(sidePlane);
      }
   }

   private final Point3d intersectionToIgnore = new Point3d();
   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      checkIfInside(x, y, heightAt, intersectionToIgnore, normalToPack);
      return heightAt;
   }

   private final Point3d tempPlaneCentroid = new Point3d();
   private final Vector3d tempPlaneNormal = new Vector3d();
   @Override
   public double heightAt(double x, double y, double z)
   {
      if (convexPolygon.isPointInside(x, y, EPSILON))
      {
         topPlane.getPoint(tempPlaneCentroid);
         topPlane.getNormal(tempPlaneNormal);

         return tempPlaneCentroid.getZ() - (tempPlaneNormal.getX() * (x - tempPlaneCentroid.getX()) + tempPlaneNormal.getY() * (y - tempPlaneCentroid.getY())) / tempPlaneNormal.getZ();
      }

      return boundingBox.getZMin();
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInside(x, y, z);
   }

//   private void closestIntersectionTo(double x, double y, double z, Point3d intersectionToPack)
//   {
//      closestIntersectionAndNormalAt(x, y, z, intersectionToPack, null);
//   }

   public boolean isInsideTheFace(Plane3d facePlane, ArrayList<Point3d> faceVertices3d, Point3d PointOnThePlane)
   {
      // Create 2d frame reference for the plane
      // The Origin of the reference frame is facePlane.point
      // The i axis is oriented as (faceVertices3d(0)-facePlane.point) vector
      // The j axis is oriented as facePlane.normal X i
      Vector3d OriginIJ = new Vector3d(facePlane.getPointCopy());
      Vector3d iVersor = new Vector3d(faceVertices3d.get(0));
      iVersor.sub(OriginIJ);
      iVersor.normalize();
      Vector3d jVersor = new Vector3d();
      jVersor.cross(facePlane.getNormalCopy(), iVersor);
      jVersor.normalize();

      // Convert 3d points in 2d points
      ArrayList<Point2d> faceVertices2d = new ArrayList<Point2d>();
      Vector3d PO = new Vector3d();
      double u;    // coordinate i
      double v;    // coordinate j

      // convert face vertices
      for (Point3d vertex : faceVertices3d)
      {
         PO.set(vertex);
         PO.sub(OriginIJ);
         u = PO.dot(iVersor);
         v = PO.dot(jVersor);
         faceVertices2d.add(new Point2d(u, v));
      }

      // Convert the point to check
      PO.set(PointOnThePlane);
      PO.sub(OriginIJ);
      u = PO.dot(iVersor);
      v = PO.dot(jVersor);
      Point2d pointToCheck = new Point2d(u, v);

      // Check if it is inside
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(faceVertices2d);

      return convexPolygon.isPointInside(pointToCheck);
   }

//   private void surfaceNormalAt(double x, double y, double z, Vector3d normalToPack)
//   {
//      closestIntersectionAndNormalAt(x, y, z, null, normalToPack);
//
//   }

//   private void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
   // FIXME It doesn't work if one of the face have Area = 0
      Point3d pointToCheck = new Point3d(x, y, z);
      ArrayList<Point3d> lowerVertices = new ArrayList<Point3d>();
      ArrayList<Point3d> upperVertices = new ArrayList<Point3d>();
      ArrayList<Point3d> faceVertices = new ArrayList<Point3d>();
      double height;
      Point3d projectedPoint = new Point3d();
      Plane3d face;
      int i;
      ArrayList<Boolean> lateralFacesLookingThePoint = new ArrayList<Boolean>();
      boolean topFaceLookingThePoint = false;
      double temporaryEdgeDistance;
      double smallestEdgeDistance = Double.MAX_VALUE;
      Point3d projectionOnEdge = new Point3d();
      LineSegment3d temporaryEdge = new LineSegment3d();
      for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
      {
         Point2d vertex = convexPolygon.getVertex(j);
         height = heightAt(vertex.getX(), vertex.getY(), 0.0);
         lowerVertices.add(new Point3d(vertex.getX(), vertex.getY(), boundingBox.getZMin()));
         upperVertices.add(new Point3d(vertex.getX(), vertex.getY(), height));
      }

      // Check on the lateral surfaces
      for (i = 0; i < sidePlanes.size(); i++)
      {
         face = new Plane3d(sidePlanes.get(i));
         lateralFacesLookingThePoint.add(face.isOnOrAbove(pointToCheck));

            projectedPoint.set(face.orthogonalProjectionCopy(pointToCheck));
            faceVertices.add(lowerVertices.get(i));
            faceVertices.add(lowerVertices.get((i + 1) % upperVertices.size()));
            faceVertices.add(upperVertices.get((i + 1) % upperVertices.size()));
            faceVertices.add(upperVertices.get(i));

            if (isInsideTheFace(face, faceVertices, projectedPoint) && projectedPoint.distance(pointToCheck) < smallestEdgeDistance)
            {
               if(intersectionToPack != null)
                  intersectionToPack.set(projectedPoint);
               if(normalToPack != null)
                  normalToPack.set(face.getNormalCopy());
               smallestEdgeDistance = projectedPoint.distance(pointToCheck);
            }

            faceVertices.clear();
      }

      // Check on the top surface
      topFaceLookingThePoint = topPlane.isOnOrAbove(pointToCheck); // FIXME Shouldn't only check if it is above; closest point could be on the top surface if it is slightly below.

      projectedPoint.set(topPlane.orthogonalProjectionCopy(pointToCheck));

      if (isInsideTheFace(topPlane, upperVertices, projectedPoint) && projectedPoint.distance(pointToCheck) < smallestEdgeDistance)
      {
         if(intersectionToPack != null)
            intersectionToPack.set(projectedPoint);
         if(normalToPack != null)
            normalToPack.set(topPlane.getNormalCopy());
         return true;
      }

      if(smallestEdgeDistance < Double.MAX_VALUE)
         return true;

      // Check edges
      for (i = 0; i < lateralFacesLookingThePoint.size(); i++)
      {
         // Check lateral edges
         if ((lateralFacesLookingThePoint.get(i) == true)
                 && (lateralFacesLookingThePoint.get((lateralFacesLookingThePoint.size() + i - 1) % lateralFacesLookingThePoint.size()) == true))
         {
            temporaryEdge.set(upperVertices.get(i), lowerVertices.get(i));
            temporaryEdgeDistance = temporaryEdge.distance(pointToCheck);

            if (temporaryEdgeDistance < smallestEdgeDistance)
            {
               smallestEdgeDistance = temporaryEdgeDistance;
               projectionOnEdge.set(temporaryEdge.orthogonalProjectionCopy(pointToCheck));
            }
         }

         // Check edges of the top face
         if ((lateralFacesLookingThePoint.get(i) == true) && (topFaceLookingThePoint == true))
         {
            temporaryEdge.set(upperVertices.get(i), upperVertices.get((i + 1) % upperVertices.size()));
            temporaryEdgeDistance = temporaryEdge.distance(pointToCheck);

            if (temporaryEdgeDistance < smallestEdgeDistance)
            {
               smallestEdgeDistance = temporaryEdgeDistance;
               projectionOnEdge.set(temporaryEdge.orthogonalProjectionCopy(pointToCheck));
            }
         }
      }

      if(intersectionToPack != null)
         intersectionToPack.set(projectionOnEdge);

      if(normalToPack != null)
      {
         normalToPack.set(pointToCheck);
         normalToPack.sub(projectionOnEdge);
         normalToPack.normalize();
      }

      if(smallestEdgeDistance < Double.MAX_VALUE)
         return true;

      return false;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

}

