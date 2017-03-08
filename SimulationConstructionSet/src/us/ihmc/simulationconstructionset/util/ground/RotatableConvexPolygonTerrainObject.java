package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
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

   public RotatableConvexPolygonTerrainObject(Vector3D normal, ConvexPolygon2d convexPolygon, double centroidHeight)
   {
      this(normal, convexPolygon, centroidHeight, YoAppearance.StoneTexture());
   }

   public RotatableConvexPolygonTerrainObject(Vector3D normal, ConvexPolygon2d convexPolygon, double centroidHeight, AppearanceDefinition appearance)
   {
      if (normal.getZ() <= 0.0)
         throw new RuntimeException("Top surface normal must have a positive z-value. Normal.z = " + normal.getZ());
      this.convexPolygon = new ConvexPolygon2d(convexPolygon);
      Point3D centroid = new Point3D(convexPolygon.getCentroid().getX(), convexPolygon.getCentroid().getY(), centroidHeight);
      this.topPlane = new Plane3d(centroid, normal);

      BoundingBox2d polygonBoundingBox = convexPolygon.getBoundingBoxCopy();
      double highest = Double.NEGATIVE_INFINITY;
      double lowest = 0.0;

      double height;
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = convexPolygon.getVertex(i);
         height = heightAt(vertex.getX(), vertex.getY(), 0.0);
         if (height < lowest)
            lowest = height;
         if (height > highest)
            highest = height;
      }

      Point2DReadOnly minPoint = polygonBoundingBox.getMinPoint();
      Point2DReadOnly maxPoint = polygonBoundingBox.getMaxPoint();
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

      Point2DReadOnly firstPoint;
      Point2DReadOnly secondPoint;
      Point3D[] topPlanePoints = new Point3D[convexPolygon.getNumberOfVertices()];
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point3D[] sidePlanePoints = new Point3D[4];

         firstPoint = convexPolygon.getVertexCCW(i);
         secondPoint = convexPolygon.getNextVertexCCW(i);

         sidePlanePoints[0] = new Point3D(firstPoint.getX(), firstPoint.getY(), lowValue);
         sidePlanePoints[1] = new Point3D(secondPoint.getX(), secondPoint.getY(), lowValue);
         sidePlanePoints[2] = new Point3D(secondPoint.getX(), secondPoint.getY(), heightAt(secondPoint.getX(), secondPoint.getY(), 0.0));
         sidePlanePoints[3] = new Point3D(firstPoint.getX(), firstPoint.getY(), heightAt(firstPoint.getX(), firstPoint.getY(), 0.0));

         this.linkGraphics.addPolygon(sidePlanePoints, appearance);

         topPlanePoints[i] = new Point3D(sidePlanePoints[3]);
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
      linkGraphics.translate(new Vector3D(plane.getPointCopy()));
      linkGraphics.addSphere(0.005, YoAppearance.Black());

      Vector3D normalCopy = plane.getNormalCopy();
      normalCopy.scale(0.01);
      linkGraphics.translate(normalCopy);
      linkGraphics.addSphere(0.005, YoAppearance.Blue());
   }

   private void initSidePlanes()
   {
      Vector2D normal2d = new Vector2D();

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         ConvexPolygon2dCalculator.getEdgeNormal(i, normal2d, convexPolygon);
         Vector3D normal = new Vector3D(normal2d.getX(), normal2d.getY(), 0.0);

         int secondIndex = (i + 1 < convexPolygon.getNumberOfVertices()) ? i + 1 : 0;
         int[] indices = {i, secondIndex};

         Point3D centerPoint = new Point3D();
         Point3D checkingPoint = new Point3D();

         for (int j : indices)
         {
            Point2DReadOnly vertex = convexPolygon.getVertex(j);
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

   private final Point3D intersectionToIgnore = new Point3D();
   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      checkIfInside(x, y, heightAt, intersectionToIgnore, normalToPack);
      return heightAt;
   }

   private final Point3D tempPlaneCentroid = new Point3D();
   private final Vector3D tempPlaneNormal = new Vector3D();
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

//   private void closestIntersectionTo(double x, double y, double z, Point3D intersectionToPack)
//   {
//      closestIntersectionAndNormalAt(x, y, z, intersectionToPack, null);
//   }

   public boolean isInsideTheFace(Plane3d facePlane, ArrayList<Point3D> faceVertices3d, Point3D PointOnThePlane)
   {
      // Create 2d frame reference for the plane
      // The Origin of the reference frame is facePlane.point
      // The i axis is oriented as (faceVertices3d(0)-facePlane.point) vector
      // The j axis is oriented as facePlane.normal X i
      Vector3D OriginIJ = new Vector3D(facePlane.getPointCopy());
      Vector3D iVersor = new Vector3D(faceVertices3d.get(0));
      iVersor.sub(OriginIJ);
      iVersor.normalize();
      Vector3D jVersor = new Vector3D();
      jVersor.cross(facePlane.getNormalCopy(), iVersor);
      jVersor.normalize();

      // Convert 3d points in 2d points
      ArrayList<Point2D> faceVertices2d = new ArrayList<Point2D>();
      Vector3D PO = new Vector3D();
      double u;    // coordinate i
      double v;    // coordinate j

      // convert face vertices
      for (Point3D vertex : faceVertices3d)
      {
         PO.set(vertex);
         PO.sub(OriginIJ);
         u = PO.dot(iVersor);
         v = PO.dot(jVersor);
         faceVertices2d.add(new Point2D(u, v));
      }

      // Convert the point to check
      PO.set(PointOnThePlane);
      PO.sub(OriginIJ);
      u = PO.dot(iVersor);
      v = PO.dot(jVersor);
      Point2D pointToCheck = new Point2D(u, v);

      // Check if it is inside
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d(faceVertices2d);

      return convexPolygon.isPointInside(pointToCheck);
   }

//   private void surfaceNormalAt(double x, double y, double z, Vector3d normalToPack)
//   {
//      closestIntersectionAndNormalAt(x, y, z, null, normalToPack);
//
//   }

//   private void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersectionToPack, Vector3d normalToPack)

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
   // FIXME It doesn't work if one of the face have Area = 0
      Point3D pointToCheck = new Point3D(x, y, z);
      ArrayList<Point3D> lowerVertices = new ArrayList<Point3D>();
      ArrayList<Point3D> upperVertices = new ArrayList<Point3D>();
      ArrayList<Point3D> faceVertices = new ArrayList<Point3D>();
      double height;
      Point3D projectedPoint = new Point3D();
      Plane3d face;
      int i;
      ArrayList<Boolean> lateralFacesLookingThePoint = new ArrayList<Boolean>();
      boolean topFaceLookingThePoint = false;
      double temporaryEdgeDistance;
      double smallestEdgeDistance = Double.MAX_VALUE;
      Point3D projectionOnEdge = new Point3D();
      LineSegment3D temporaryEdge = new LineSegment3D();
      for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
      {
         Point2DReadOnly vertex = convexPolygon.getVertex(j);
         height = heightAt(vertex.getX(), vertex.getY(), 0.0);
         lowerVertices.add(new Point3D(vertex.getX(), vertex.getY(), boundingBox.getZMin()));
         upperVertices.add(new Point3D(vertex.getX(), vertex.getY(), height));
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
               projectionOnEdge.set(temporaryEdge.orthogonalProjection(pointToCheck));
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
               projectionOnEdge.set(temporaryEdge.orthogonalProjection(pointToCheck));
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

