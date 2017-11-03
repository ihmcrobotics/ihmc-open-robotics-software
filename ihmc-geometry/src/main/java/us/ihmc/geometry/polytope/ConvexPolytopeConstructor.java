package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;

public class ConvexPolytopeConstructor
{
   public enum Direction
   {
      AlongAxis, OppositeAxis;

      public double getSign()
      {
         switch (this)
         {
         case AlongAxis:
            return 1.0;
         case OppositeAxis:
            return -1.0;
         default:
            throw new RuntimeException("Unknown case");
         }
      }
   }

   private static final double EPSILON = Epsilons.ONE_BILLIONTH;
   private static final Point3D origin = new Point3D();

   public static ConvexPolytope constructUnitCube()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      polytope.addVertex(0.0, 0.0, 0.0);
      polytope.addVertex(1.0, 0.0, 0.0);
      polytope.addVertex(0.0, 1.0, 0.0);
      polytope.addVertex(1.0, 1.0, 0.0);
      polytope.addVertex(0.0, 0.0, 1.0);
      polytope.addVertex(1.0, 0.0, 1.0);
      polytope.addVertex(0.0, 1.0, 1.0);
      polytope.addVertex(1.0, 1.0, 1.0);
      return polytope;
   }

   public static ExtendedConvexPolytope constructExtendedUnitCube()
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      polytope.addVertex(new ExtendedPolytopeVertex(0.0, 0.0, 0.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(1.0, 0.0, 0.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(0.0, 1.0, 0.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(1.0, 1.0, 0.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(0.0, 0.0, 1.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(1.0, 0.0, 1.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(0.0, 1.0, 1.0), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(1.0, 1.0, 1.0), EPSILON);
      return polytope;
   }

   public static ConvexPolytope constructBox(Point3D center, Quaternion orientation, double edgeLengthX, double edgeLengthY, double edgeLengthZ)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(orientation);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2);
      polytope.applyTransform(transform);
      return polytope;
   }

   public static ExtendedConvexPolytope constructExtendedBox(Point3D center, Quaternion orientation, double edgeLengthX, double edgeLengthY, double edgeLengthZ)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(orientation);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() - edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() - edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() - edgeLengthY / 2, center.getZ() + edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() + edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2, EPSILON);
      polytope.addVertex(center.getX() - edgeLengthX / 2, center.getY() + edgeLengthY / 2, center.getZ() + edgeLengthZ / 2, EPSILON);
      polytope.applyTransform(transform);
      return polytope;
   }

   /**
    * Constructs a icosahedron that envelops the sphere to be created
    * @param radius
    * @param center
    * @param edgeLengthForDiscretization
    * @return
    */
   public static ExtendedConvexPolytope constructIcoSphere(double radius, Point3D center, int recursionLevel)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      int t1 = 1;
      double t2 = 1 + Math.sqrt(5) / 2;
      double scale = radius / Math.sqrt(t1 * t1 + t2 * t2);
      t1 *= scale;
      t2 *= scale;

      polytope.addVertex(center.getX() - t1, center.getY() + t2, center.getZ(), EPSILON);
      polytope.addVertex(center.getX() + t1, center.getY() + t2, center.getZ(), EPSILON);
      polytope.addVertex(center.getX() - t1, center.getY() - t2, center.getZ(), EPSILON);
      polytope.addVertex(center.getX() + t1, center.getY() - t2, center.getZ(), EPSILON);

      polytope.addVertex(center.getX(), center.getY() - t1, center.getZ() + t2, EPSILON);
      polytope.addVertex(center.getX(), center.getY() + t1, center.getZ() + t2, EPSILON);
      polytope.addVertex(center.getX(), center.getY() - t1, center.getZ() - t2, EPSILON);
      polytope.addVertex(center.getX(), center.getY() + t1, center.getZ() - t2, EPSILON);

      polytope.addVertex(center.getX() + t2, center.getY(), center.getZ() - t1, EPSILON);
      polytope.addVertex(center.getX() + t2, center.getY(), center.getZ() + t1, EPSILON);
      polytope.addVertex(center.getX() - t2, center.getY(), center.getZ() - t1, EPSILON);
      polytope.addVertex(center.getX() - t2, center.getY(), center.getZ() + t1, EPSILON);

      // FIXME add the recursion level code here. Mostly need to precompute the points and then add to polytope. But then whats the point of having the polytope class
      //      List<PolytopeHalfEdge> edges = new ArrayList<>((int) (120 * Math.pow(4, recursionLevel)));
      //      for (int i = 0; i < recursionLevel; i++)
      //      {
      //         edges.clear();
      //         edges.addAll(polytope.getEdges());
      //         scale = radius / Math.sqrt(radius * radius - edges.get(0).getEdgeVector().dot(edges.get(0).getEdgeVector())) / 2.0;
      //         for (int j = 0; j < edges.size();)
      //         {
      //            PrintTools.debug(j + "");
      //            PolytopeVertex origin = edges.get(j).getOriginVertex();
      //            PolytopeVertex destination = edges.get(j).getDestinationVertex();
      //            PrintTools.debug(origin == null ? "null" : origin.toString());
      //            PrintTools.debug(destination == null ? "null" : origin.toString());
      //            PolytopeVertex newVertex = new PolytopeVertex((origin.getX() + destination.getX()) * scale, (origin.getY() + destination.getY()) * scale,
      //                                                          (origin.getZ() + destination.getZ()) * scale);
      //            polytope.addVertex(newVertex, EPSILON);
      //            edges.remove(edges.get(j));
      //         }
      //      }
      return polytope;
   }

   public static ExtendedConvexPolytope constructSphere(Point3D center, double radius, int cubeDivisions)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      List<ExtendedPolytopeVertex> vertices = new ArrayList<>();
      for (int i = 0; i < cubeDivisions; i++)
      {
         for (int j = 0; j < cubeDivisions; j++)
         {
            ExtendedPolytopeVertex vertex = new ExtendedPolytopeVertex((2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius,
                                                                       (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, -radius);
            vertices.add(vertex);
         }
      }

      for (int i = 1; i < cubeDivisions; i++)
      {
         for (int j = 0; j < cubeDivisions; j++)
         {
            ExtendedPolytopeVertex vertex = new ExtendedPolytopeVertex(-radius, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius,
                                                                       (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new ExtendedPolytopeVertex((2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, -radius,
                                                (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new ExtendedPolytopeVertex((2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, radius,
                                                (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
            vertex = new ExtendedPolytopeVertex(radius, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius,
                                                (2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius);
            vertices.add(vertex);
         }
      }

      for (int i = 1; i < cubeDivisions - 1; i++)
      {
         for (int j = 1; j < cubeDivisions - 1; j++)
         {
            ExtendedPolytopeVertex vertex = new ExtendedPolytopeVertex((2.0 * (float) i / (float) (cubeDivisions - 1) - 1) * radius,
                                                                       (2.0 * (float) j / (float) (cubeDivisions - 1) - 1) * radius, radius);
            vertices.add(vertex);
         }
      }

      for (int i = 0; i < vertices.size(); i++)
      {
         ExtendedPolytopeVertex vertex = vertices.get(i);
         double mag = Math.sqrt(vertex.getX() * vertex.getX() + vertex.getY() * vertex.getY() + vertex.getZ() * vertex.getZ());
         vertex.setX(vertex.getX() * radius / mag);
         vertex.setY(vertex.getY() * radius / mag);
         vertex.setZ(vertex.getZ() * radius / mag);
         polytope.addVertex(vertex, EPSILON);
      }
      return polytope;
   }

   public static ExtendedConvexPolytope constructUnitSphere(int recursionLevel)
   {
      return constructIcoSphere(1.0, new Point3D(), recursionLevel);
   }

   /**
    * Creates a polytope by discretizing the curved surface of the cylinder
    * @param center location of the polytope centroid
    * @param radius radius of the cylinder to be made
    * @param length lenght of the cylinder to be made
    * @param numberOfDivisionsForCurvedSurface
    * @return
    */
   public static ExtendedConvexPolytope constructCylinder(Point3D center, double radius, double length, int numberOfDivisionsForCurvedSurface)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      double vertexAngle = 2 * Math.PI / numberOfDivisionsForCurvedSurface;
      double enclosingRadius = radius / Math.cos(vertexAngle / 2.0);
      for (int i = 0; i < numberOfDivisionsForCurvedSurface; i++)
         polytope.addVertex(new ExtendedPolytopeVertex(center.getX() + enclosingRadius * Math.cos(i * vertexAngle),
                                                       center.getY() + enclosingRadius * Math.sin(i * vertexAngle), center.getZ() - length / 2.0),
                            EPSILON);
      for (int i = 0; i < numberOfDivisionsForCurvedSurface; i++)
      {
         polytope.addVertex(new ExtendedPolytopeVertex(center.getX() + enclosingRadius * Math.cos(i * vertexAngle),
                                                       center.getY() + enclosingRadius * Math.sin(i * vertexAngle), center.getZ() + length / 2.0),
                            EPSILON);
      }
      return polytope;
   }

   public static FrameConvexPolytope constructUnitCube(ReferenceFrame frame)
   {
      FrameConvexPolytope polytope = new FrameConvexPolytope(frame, constructExtendedUnitCube());
      return polytope;
   }

   public static ConvexPolytope constructBoxWithCenterAtZero(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      polytope.addVertex(-halfLengthX, -halfWidthY, -halfHeightZ);
      polytope.addVertex(halfLengthX, -halfWidthY, -halfHeightZ);
      polytope.addVertex(halfLengthX, halfWidthY, -halfHeightZ);
      polytope.addVertex(-halfLengthX, halfWidthY, -halfHeightZ);
      polytope.addVertex(-halfLengthX, -halfWidthY, halfHeightZ);
      polytope.addVertex(halfLengthX, -halfWidthY, halfHeightZ);
      polytope.addVertex(halfLengthX, halfWidthY, halfHeightZ);
      polytope.addVertex(-halfLengthX, halfWidthY, halfHeightZ);

      return polytope;
   }

   public static ExtendedConvexPolytope constructExtendedBoxWithCenterAtZero(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();

      polytope.addVertex(new ExtendedPolytopeVertex(-halfLengthX, -halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(halfLengthX, -halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(halfLengthX, halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(-halfLengthX, halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(-halfLengthX, -halfWidthY, halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(halfLengthX, -halfWidthY, halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(halfLengthX, halfWidthY, halfHeightZ), EPSILON);
      polytope.addVertex(new ExtendedPolytopeVertex(-halfLengthX, halfWidthY, halfHeightZ), EPSILON);

      return polytope;
   }

   public static ConvexPolytope constructRandomSphereOutlinedPolytope(Random random, int numberOfPoints, double radius, double xyzBoundary)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      Point3D sphereCenter = EuclidCoreRandomTools.generateRandomPoint3D(random, xyzBoundary, xyzBoundary, xyzBoundary);
      for (int i = 0; i < numberOfPoints; i++)
      {
         Vector3D randomVector = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, radius);
         Point3D point = new Point3D(sphereCenter);
         point.add(randomVector);
         polytope.addVertex(point);
      }

      return polytope;
   }

   public static ExtendedConvexPolytope constructExtendedRandomSphereOutlinedPolytope(Random random, int numberOfPoints, double radius, double xyzBoundary)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();

      Point3D sphereCenter = EuclidCoreRandomTools.generateRandomPoint3D(random, xyzBoundary, xyzBoundary, xyzBoundary);
      for (int i = 0; i < numberOfPoints; i++)
      {
         Vector3D randomVector = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, radius);
         Point3D point = new Point3D(sphereCenter);
         point.add(randomVector);
         polytope.addVertex(point, EPSILON);
      }

      return polytope;
   }

   public static ConvexPolytope constructSinglePointPolytope(Point3D singlePoint)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      polytope.addVertex(singlePoint);
      return polytope;
   }

   public static ExtendedConvexPolytope constructSinglePointExtendedPolytope(Point3D singlePoint)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      polytope.addVertex(singlePoint, EPSILON);
      return polytope;
   }

   public static ConvexPolytope constructFromVertices(double[][] vertices)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      for (double[] vertex : vertices)
      {
         polytope.addVertex(vertex);
      }
      return polytope;
   }

   public static ExtendedConvexPolytope constructExtendedFromVertices(double[][] vertices)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      for (double[] vertex : vertices)
      {
         polytope.addVertex(EPSILON, vertex);
      }
      return polytope;
   }

   /////// All the code above this is a mess that exists for some reason that I dont want to get into right now 
   public static FrameConvexPolytope getFrameSphericalCollisionMeshByProjectingCube(FramePoint3D centroid, double radius, int cubeDivisions)
   {
      return createFramePolytope(centroid.getReferenceFrame(), getCollisionMeshPointsForSphere(centroid.getPoint(), radius, cubeDivisions));
   }

   public static FrameConvexPolytope getFrameSphericalCollisionMeshByProjectingCube(ReferenceFrame referenceFrame, Point3D centroid, double radius,
                                                                                    int cubeDivisions)
   {
      return createFramePolytope(referenceFrame, getCollisionMeshPointsForSphere(centroid, radius, cubeDivisions));
   }

   public static ExtendedConvexPolytope getSphericalCollisionMeshByProjectingCube(Point3D centroid, double radius, int cubeDivisions)
   {
      return createPolytope(getCollisionMeshPointsForSphere(centroid, radius, cubeDivisions));
   }

   public static ExtendedConvexPolytope createPolytope(ArrayList<Point3D> pointsToAdd)
   {
      ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
      addVerticesToPolytope(polytope, pointsToAdd);
      return polytope;
   }

   public static FrameConvexPolytope createFramePolytope(ReferenceFrame referenceFrame, ArrayList<Point3D> pointsToAdd)
   {
      FrameConvexPolytope polytope = new FrameConvexPolytope(referenceFrame);
      addVerticesToPolytope(polytope, pointsToAdd);
      return polytope;
   }

   public static void addVerticesToPolytope(ConvexPolytopeBasics polytope, ArrayList<Point3D> verticesToAdd)
   {
      for (int i = 0; i < verticesToAdd.size(); i++)
         polytope.addVertex(verticesToAdd.get(i), EPSILON);
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForSphere(Point3D center, double radius, int cubeDivisions)
   {
      return getCollisionMeshPointsForSphere(center.getX(), center.getY(), center.getZ(), radius, cubeDivisions);
   }

   public static void getCollisionMeshPointsForSphere(RigidBodyTransform transform, double radius, int cubeDivisions, ArrayList<Point3D> pointsToPack)
   {
      getPointsOnUnitCubeForSphereGeneration(cubeDivisions, pointsToPack);
      projectPointsToRadius(radius, pointsToPack);
      applyTransformToPoints(transform, pointsToPack);
   }

   public static void applyTransformToPoints(Transform transform, ArrayList<Point3D> points)
   {
      for (int i = 0; i < points.size(); i++)
         points.get(i).applyTransform(transform);
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForSphere(double centroidX, double centroidY, double centroidZ, double radius, int cubeDivisions)
   {
      ArrayList<Point3D> points = new ArrayList<>();
      getCollisionMeshPointsForSphere(centroidX, centroidY, centroidZ, radius, cubeDivisions, points);
      return points;
   }

   public static void getCollisionMeshPointsForSphere(double centroidX, double centroidY, double centroidZ, double radius, int cubeDivisions,
                                                      ArrayList<Point3D> pointsToPack)
   {
      getPointsOnUnitCubeForSphereGeneration(cubeDivisions, pointsToPack);
      projectPointsToRadiusAndShiftCentroid(centroidX, centroidY, centroidZ, radius, pointsToPack);
   }

   public static void projectPointsToRadiusAndShiftCentroid(double centroidX, double centroidY, double centroidZ, double radius,
                                                            ArrayList<Point3D> pointsToPack)
   {
      projectPointsToRadius(radius, pointsToPack);
      shiftCentroid(centroidX, centroidY, centroidZ, pointsToPack);
   }

   public static void projectPointsToRadius(double radius, ArrayList<Point3D> pointsToPack)
   {
      for (int i = 0; i < pointsToPack.size(); i++)
      {
         Point3D point = pointsToPack.get(i);
         double scalar = radius / point.distanceFromOrigin();
         point.scale(scalar);
      }
   }

   public static void shiftCentroid(double centroidX, double centroidY, double centroidZ, ArrayList<Point3D> pointsToPack)
   {
      for (int i = 0; i < pointsToPack.size(); i++)
         pointsToPack.get(i).add(centroidX, centroidY, centroidZ);
   }

   public static void getPointsOnUnitCubeForSphereGeneration(int cubeDivisions, ArrayList<Point3D> pointsToPack)
   {
      for (int i = 0; i < cubeDivisions; i++)
      {
         double x = (2.0 * (float) i / (float) (cubeDivisions - 1) - 1);
         for (int j = 0; j < cubeDivisions; j++)
         {
            pointsToPack.add(new Point3D(x, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1), -1.0));
         }
      }

      for (int i = 1; i < cubeDivisions; i++)
      {
         double z = (2.0 * (float) i / (float) (cubeDivisions - 1) - 1);
         for (int j = 0; j < cubeDivisions; j++)
         {
            double xy = (2.0 * (float) j / (float) (cubeDivisions - 1) - 1);
            pointsToPack.add(new Point3D(-1.0, xy, z));
            pointsToPack.add(new Point3D(xy, -1.0, z));
            pointsToPack.add(new Point3D(xy, 1.0, z));
            pointsToPack.add(new Point3D(1.0, xy, z));
         }
      }

      for (int i = 1; i < cubeDivisions - 1; i++)
      {
         double x = (2.0 * (float) i / (float) (cubeDivisions - 1) - 1);
         for (int j = 1; j < cubeDivisions - 1; j++)
         {
            pointsToPack.add(new Point3D(x, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1), 1.0));
         }
      }
   }

   public static void getCollisionMeshPointsForIcoSphere(Point3D centroid, double radius, int cubeDivisions, ArrayList<Point3D> pointsToPack)
   {
      throw new RuntimeException("Not implemented");
   }

   public static void getCylindericalCollisionMesh(Transform transform, double radius, double length, int curvedSurfaceDivisions,
                                                   ArrayList<Point3D> pointsToPack)
   {
      getCollisionMeshPointsForCylinder(0.0, 0.0, 0.0, Axis.Z, radius, length, curvedSurfaceDivisions, pointsToPack);
      applyTransformToPoints(transform, pointsToPack);
   }

   public static FrameConvexPolytope getFrameCylindericalCollisionMesh(FramePoint3D centroid, Axis axis, double radius, double length,
                                                                       int curvedSurfaceDivisions)
   {
      return createFramePolytope(centroid.getReferenceFrame(),
                                 getCollisionMeshPointsForCylinder(centroid.getPoint(), axis, radius, length, curvedSurfaceDivisions));
   }

   public static FrameConvexPolytope getFrameCylindericalCollisionMesh(ReferenceFrame referenceFrame, Point3D centroid, Axis axis, double radius, double length,
                                                                       int curvedSurfaceDivisions)
   {
      return createFramePolytope(referenceFrame, getCollisionMeshPointsForCylinder(centroid, axis, radius, length, curvedSurfaceDivisions));
   }

   public static ExtendedConvexPolytope getCylindericalCollisionMesh(Point3D centroid, Axis axis, double radius, double length, int curvedSurfaceDivisions)
   {
      return createPolytope(getCollisionMeshPointsForCylinder(centroid, axis, radius, length, curvedSurfaceDivisions));
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForCylinder(Point3D centroid, Axis axis, double radius, double length, int curvedSurfaceDivisions)
   {
      return getCollisionMeshPointsForCylinder(centroid.getX(), centroid.getY(), centroid.getZ(), axis, radius, length, curvedSurfaceDivisions);
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForCylinder(double centroidX, double centroidY, double centroidZ, Axis axis, double radius,
                                                                      double length, int curvedSurfaceDivisions)
   {
      ArrayList<Point3D> points = new ArrayList<>();
      getCollisionMeshPointsForCylinder(centroidX, centroidY, centroidZ, axis, radius, length, curvedSurfaceDivisions, points);
      return points;
   }

   public static void getCollisionMeshPointsForCylinder(double centroidX, double centroidY, double centroidZ, Axis axis, double radius, double length,
                                                        int curvedSurfaceDivisions, ArrayList<Point3D> pointsToPack)
   {
      getCollisionMeshPointsForCylinder(centroidX, centroidY, centroidZ, axis, radius, length, curvedSurfaceDivisions, false, pointsToPack);
   }

   public static void getCollisionMeshPointsForCylinder(double centroidX, double centroidY, double centroidZ, Axis axis, double radius, double length,
                                                        int curvedSurfaceDivisions, boolean getEnclosingPoints, ArrayList<Point3D> pointsToPack)
   {
      double vertexAngle = 2 * Math.PI / curvedSurfaceDivisions;
      double enclosingRadius = radius;
      if (getEnclosingPoints)
         enclosingRadius /= Math.cos(vertexAngle / 2.0);

      switch (axis)
      {
      case X:
         for (int i = 0; i < curvedSurfaceDivisions; i++)
            pointsToPack.add(new Point3D(centroidX - length / 2.0, centroidY + enclosingRadius * Math.sin(i * vertexAngle),
                                         centroidZ + enclosingRadius * Math.cos(i * vertexAngle)));
         for (int i = 0; i < curvedSurfaceDivisions; i++)
            pointsToPack.add(new Point3D(centroidX + length / 2.0, centroidY + enclosingRadius * Math.sin(i * vertexAngle),
                                         centroidZ + enclosingRadius * Math.cos(i * vertexAngle)));
         break;
      case Y:
         for (int i = 0; i < curvedSurfaceDivisions; i++)
            pointsToPack.add(new Point3D(centroidX + enclosingRadius * Math.cos(i * vertexAngle), centroidY - length / 2.0,
                                         centroidZ + enclosingRadius * Math.sin(i * vertexAngle)));
         for (int i = 0; i < curvedSurfaceDivisions; i++)
            pointsToPack.add(new Point3D(centroidX + enclosingRadius * Math.cos(i * vertexAngle), centroidY + length / 2.0,
                                         centroidZ + enclosingRadius * Math.sin(i * vertexAngle)));
         break;
      default:
         for (int i = 0; i < curvedSurfaceDivisions; i++)
            pointsToPack.add(new Point3D(centroidX + enclosingRadius * Math.cos(i * vertexAngle), centroidY + enclosingRadius * Math.sin(i * vertexAngle),
                                         centroidZ - length / 2.0));
         for (int i = 0; i < curvedSurfaceDivisions; i++)
            pointsToPack.add(new Point3D(centroidX + enclosingRadius * Math.cos(i * vertexAngle), centroidY + enclosingRadius * Math.sin(i * vertexAngle),
                                         centroidZ + length / 2.0));
         break;
      }
   }

   public static void getCuboidCollisionMesh(Transform transform, double xLength, double yLength, double zLength, ArrayList<Point3D> pointsToPack)
   {
      getCollisionMeshPointsForCuboid(0.0, 0.0, 0.0, xLength, yLength, zLength, pointsToPack);
      applyTransformToPoints(transform, pointsToPack);
   }

   public static FrameConvexPolytope getFrameCuboidCollisionMesh(FramePoint3D centroid, double xLength, double yLength, double zLength)
   {
      return createFramePolytope(centroid.getReferenceFrame(), getCollisionMeshPointsForCuboid(centroid.getPoint(), xLength, yLength, zLength));
   }

   public static FrameConvexPolytope getFrameCuboidCollisionMesh(ReferenceFrame referenceFrame, Point3D centroid, double xLength, double yLength,
                                                                 double zLength)
   {
      return createFramePolytope(referenceFrame, getCollisionMeshPointsForCuboid(centroid, xLength, yLength, zLength));
   }

   public static ExtendedConvexPolytope getCuboidCollisionMesh(Point3D centroid, double xLength, double yLength, double zLength)
   {
      return createPolytope(getCollisionMeshPointsForCuboid(centroid, xLength, yLength, zLength));
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForCuboid(Point3D centroid, double xLength, double yLength, double zLength)
   {
      return getCollisionMeshPointsForCuboid(centroid.getX(), centroid.getY(), centroid.getZ(), xLength, yLength, zLength);
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForCuboid(double centroidX, double centroidY, double centroidZ, double xLength, double yLength,
                                                                    double zLength)
   {
      ArrayList<Point3D> points = new ArrayList<>();
      getCollisionMeshPointsForCuboid(centroidX, centroidY, centroidZ, xLength, yLength, zLength, points);
      return points;
   }

   public static void getCollisionMeshPointsForCuboid(double centroidX, double centroidY, double centroidZ, double xLength, double yLength, double zLength,
                                                      ArrayList<Point3D> pointsToPack)
   {
      double halfLengthX = xLength / 2.0;
      double halfWidthY = yLength / 2.0;
      double halfHeightZ = zLength / 2.0;
      double negativeXCoord = -halfLengthX + centroidX;
      double positiveXCoord = halfLengthX + centroidX;
      double negativeYCoord = -halfWidthY + centroidY;
      double positiveYCoord = halfWidthY + centroidY;
      double negativeZCoord = -halfHeightZ + centroidZ;
      double positiveZCoord = halfHeightZ + centroidZ;
      pointsToPack.add(new Point3D(negativeXCoord, negativeYCoord, negativeZCoord));
      pointsToPack.add(new Point3D(positiveXCoord, negativeYCoord, negativeZCoord));
      pointsToPack.add(new Point3D(positiveXCoord, positiveYCoord, negativeZCoord));
      pointsToPack.add(new Point3D(negativeXCoord, positiveYCoord, negativeZCoord));
      pointsToPack.add(new Point3D(negativeXCoord, negativeYCoord, positiveZCoord));
      pointsToPack.add(new Point3D(positiveXCoord, negativeYCoord, positiveZCoord));
      pointsToPack.add(new Point3D(positiveXCoord, positiveYCoord, positiveZCoord));
      pointsToPack.add(new Point3D(negativeXCoord, positiveYCoord, positiveZCoord));
   }

   public static FrameConvexPolytope getFrameCapsuleCollisionMesh(ReferenceFrame referenceFrame, Point3D centeroid, Axis axis, double cylindericalLength,
                                                           double endRadius, int curvedSurfaceDivisions)
   {
      return createFramePolytope(referenceFrame, getCollisionMeshPointsForCapsule(centeroid, axis, cylindericalLength, endRadius, curvedSurfaceDivisions));
   }

   public static FrameConvexPolytope getFrameCapsuleCollisionMesh(FramePoint3D centeroid, Axis axis, double cylindericalLength, double endRadius,
                                                           int curvedSurfaceDivisions)
   {
      return createFramePolytope(centeroid.getReferenceFrame(),
                                 getCollisionMeshPointsForCapsule(centeroid.getPoint(), axis, cylindericalLength, endRadius, curvedSurfaceDivisions));
   }

   public static ExtendedConvexPolytope getCapsuleCollisionMesh(Point3D centeroid, Axis axis, double cylindericalLength, double endRadius, int curvedSurfaceDivisions)
   {
      return createPolytope(getCollisionMeshPointsForCapsule(centeroid, axis, cylindericalLength, endRadius, curvedSurfaceDivisions));
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForCapsule(Point3D centeroid, Axis axis, double cylindericalLength, double endRadius,
                                                                     int curvedSurfaceDivisions)
   {
      return getCollisionMeshPointsForCapsule(centeroid.getX(), centeroid.getY(), centeroid.getZ(), axis, cylindericalLength, endRadius,
                                              curvedSurfaceDivisions);
   }

   public static void getCollisionMeshPointsForCapsule(LineSegment3D lineSegment, double endRadius, int curvedSurfaceDivisions, ArrayList<Point3D> pointsToPack)
   {
      RigidBodyTransform transform = createRigidBodyTransformFromLineSegment(lineSegment);
      getCollisionMeshPointsForCapsule(0.0, 0.0, 0.0, Axis.X, getLengthOfLineSegment(lineSegment), endRadius, curvedSurfaceDivisions, pointsToPack);
      applyTransformToPoints(transform, pointsToPack);
   }

   private static double getLengthOfLineSegment(LineSegment3D lineSegment)
   {
      return EuclidCoreTools.norm(lineSegment.getFirstEndpointX() - lineSegment.getSecondEndpointX(),
                                  lineSegment.getFirstEndpointY() - lineSegment.getSecondEndpointY(),
                                  lineSegment.getFirstEndpointZ() - lineSegment.getSecondEndpointZ());
   }

   private static RigidBodyTransform createRigidBodyTransformFromLineSegment(LineSegment3D lineSegment)
   {

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      Vector3D axisAngleVector = new Vector3D();
      lineSegment.getDirection(true, axisAngleVector);
      double angle = axisAngleVector.angle(Axis.X.getAxisVector());
      axisAngleVector.cross(Axis.X.getAxisVector());
      axisAngleVector.normalize();
      axisAngleVector.scale(angle);
      rigidBodyTransform.setRotation(axisAngleVector);
      rigidBodyTransform.prependTranslation(lineSegment.getFirstEndpointX() - lineSegment.getSecondEndpointX(),
                                            lineSegment.getFirstEndpointY() - lineSegment.getSecondEndpointY(),
                                            lineSegment.getFirstEndpointZ() - lineSegment.getSecondEndpointZ());
      return rigidBodyTransform;
   }

   public static ArrayList<Point3D> getCollisionMeshPointsForCapsule(double centroidX, double centroidY, double centroidZ, Axis axis, double cylindericalLength,
                                                                     double endRadius, int curvedSurfaceDivisions)
   {
      ArrayList<Point3D> points = new ArrayList<>();
      getCollisionMeshPointsForCapsule(centroidX, centroidY, centroidZ, axis, cylindericalLength, endRadius, curvedSurfaceDivisions, points);
      return points;
   }

   private static ArrayList<Point3D> tempList1 = new ArrayList<>();
   private static ArrayList<Point3D> tempList2 = new ArrayList<>();
   
   /**
    * 
    * @param centroid
    * @param axis
    * @param cylindericalLength
    * @param endRadius
    * @param pointsToPack
    * @param curvedSurfaceDivisions will be rounded down up to the nearest multiple of four
    */
   public static void getCollisionMeshPointsForCapsule(double centroidX, double centroidY, double centroidZ, Axis axis, double cylindericalLength,
                                                       double endRadius, int curvedSurfaceDivisions, ArrayList<Point3D> pointsToPack)
   {
      curvedSurfaceDivisions = curvedSurfaceDivisions + (4 - (curvedSurfaceDivisions % 4)) % 4;
      tempList1.clear();
      tempList2.clear();
      switch (axis)
      {
      case X:
         getCollisionMeshPointsForHemisphere(centroidX + cylindericalLength / 2.0, centroidY, centroidZ, axis, Direction.OppositeAxis, endRadius,
                                             curvedSurfaceDivisions, tempList1);
         getCollisionMeshPointsForHemisphere(centroidX - cylindericalLength / 2.0, centroidY, centroidZ, axis, Direction.AlongAxis, endRadius,
                                             curvedSurfaceDivisions, tempList2);
         break;
      case Y:
         getCollisionMeshPointsForHemisphere(centroidX, centroidY + cylindericalLength / 2.0, centroidZ, axis, Direction.OppositeAxis, endRadius,
                                             curvedSurfaceDivisions, tempList1);
         getCollisionMeshPointsForHemisphere(centroidX, centroidY - cylindericalLength / 2.0, centroidZ, axis, Direction.AlongAxis, endRadius,
                                             curvedSurfaceDivisions, tempList2);
         break;
      default:
         getCollisionMeshPointsForHemisphere(centroidX, centroidY, centroidZ + cylindericalLength / 2.0, axis, Direction.OppositeAxis, endRadius,
                                             curvedSurfaceDivisions, tempList1);
         getCollisionMeshPointsForHemisphere(centroidX, centroidY, centroidZ - cylindericalLength / 2.0, axis, Direction.AlongAxis, endRadius,
                                             curvedSurfaceDivisions, tempList2);
         break;
      }
      getCollisionMeshPointsForCylinder(centroidX, centroidY, centroidZ, axis, endRadius, cylindericalLength, curvedSurfaceDivisions, pointsToPack);
      pointsToPack.addAll(tempList1);
      pointsToPack.addAll(tempList2);
   }

   public static void getCollisionMeshPointsForHemisphere(double centroidX, double centroidY, double centroidZ, Axis axis, Direction direction, double radius,
                                                          int cubeDivisions, ArrayList<Point3D> pointsToPack)
   {
      getPointsOnUnitCubeForHemisphereGeneration(axis, direction, cubeDivisions, pointsToPack);
      projectPointsToRadiusAndShiftCentroid(centroidX, centroidY, centroidZ, radius, pointsToPack);
   }

   public static void getPointsOnUnitCubeForHemisphereGeneration(Axis axis, Direction direction, int cubeDivisions, ArrayList<Point3D> pointsToPack)
   {
      for (int i = 0; i < cubeDivisions; i++)
      {
         double x = (2.0 * (float) i / (float) (cubeDivisions - 1) - 1);
         for (int j = 0; j < cubeDivisions; j++)
         {
            Point3D point = new Point3D();
            point.setElement(axis.ordinal(), -direction.getSign());
            point.setElement((axis.ordinal() + 1) % 3, x);
            point.setElement((axis.ordinal() + 2) % 3, (2.0 * (float) j / (float) (cubeDivisions - 1) - 1));
            pointsToPack.add(point);
         }
      }

      for (int i = 1; i < cubeDivisions / 2; i++)
      {
         double z = direction.getSign() * (2.0 * (float) i / (float) (cubeDivisions - 1) - 1);
         for (int j = 0; j < cubeDivisions; j++)
         {
            double xy = (2.0 * (float) j / (float) (cubeDivisions - 1) - 1);
            Point3D point = new Point3D();
            point.setElement(axis.ordinal(), z);
            point.setElement((axis.ordinal() + 1) % 3, -1.0);
            point.setElement((axis.ordinal() + 2) % 3, xy);
            pointsToPack.add(point);

            point = new Point3D();
            point.setElement(axis.ordinal(), z);
            point.setElement((axis.ordinal() + 1) % 3, xy);
            point.setElement((axis.ordinal() + 2) % 3, -1.0);
            pointsToPack.add(point);

            point = new Point3D();
            point.setElement(axis.ordinal(), z);
            point.setElement((axis.ordinal() + 1) % 3, xy);
            point.setElement((axis.ordinal() + 2) % 3, 1.0);
            pointsToPack.add(point);

            point = new Point3D();
            point.setElement(axis.ordinal(), z);
            point.setElement((axis.ordinal() + 1) % 3, 1.0);
            point.setElement((axis.ordinal() + 2) % 3, xy);
            pointsToPack.add(point);
         }
      }
   }

   public static void shiftCentroid(Vector3D centerOfMassOffset, ArrayList<Point3D> pointsForShapeDescription)
   {
      shiftCentroid(centerOfMassOffset.getX(), centerOfMassOffset.getY(), centerOfMassOffset.getZ(), pointsForShapeDescription);
   }
}
