package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import javafx.util.Pair;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class IntersectionPlaneBoxCalculator
{
   private static final double EPSILON = 1.0e-3;

   private final Point3D[] boxVertices = new Point3D[8];
   private final List<Pair<Point3D, Point3D>> boxEdges = new ArrayList<>();

   private final Vector3D boxSize = new Vector3D();
   private final Point3D boxCenter = new Point3D();

   private final Point3D pointOnPlane = new Point3D();
   private final Vector3D planeNormal = new Vector3D();

   public IntersectionPlaneBoxCalculator()
   {
      // Index ordering as in http://paulbourke.net/geometry/polygonise/
      boxVertices[0] = new Point3D( 0.5,  0.5, -0.5);
      boxVertices[1] = new Point3D( 0.5, -0.5, -0.5);
      boxVertices[2] = new Point3D(-0.5, -0.5, -0.5);
      boxVertices[3] = new Point3D(-0.5,  0.5, -0.5);
      boxVertices[4] = new Point3D( 0.5,  0.5,  0.5);
      boxVertices[5] = new Point3D( 0.5, -0.5,  0.5);
      boxVertices[6] = new Point3D(-0.5, -0.5,  0.5);
      boxVertices[7] = new Point3D(-0.5,  0.5,  0.5);

      boxEdges.add(new Pair<>(boxVertices[0], boxVertices[1]));
      boxEdges.add(new Pair<>(boxVertices[1], boxVertices[2]));
      boxEdges.add(new Pair<>(boxVertices[2], boxVertices[3]));
      boxEdges.add(new Pair<>(boxVertices[3], boxVertices[0]));
      boxEdges.add(new Pair<>(boxVertices[4], boxVertices[5]));
      boxEdges.add(new Pair<>(boxVertices[5], boxVertices[6]));
      boxEdges.add(new Pair<>(boxVertices[6], boxVertices[7]));
      boxEdges.add(new Pair<>(boxVertices[7], boxVertices[4]));
      boxEdges.add(new Pair<>(boxVertices[0], boxVertices[4]));
      boxEdges.add(new Pair<>(boxVertices[1], boxVertices[5]));
      boxEdges.add(new Pair<>(boxVertices[2], boxVertices[6]));
      boxEdges.add(new Pair<>(boxVertices[3], boxVertices[7]));
   }

   public void setCube(double size, Point3D center)
   {
      setBox(size, size, size, center);
   }

   public void setCube(double size, double centerX, double centerY, double centerZ)
   {
      setBox(size, size, size, centerX, centerY, centerZ);
   }

   public void setBox(double lx, double ly, double lz, Point3D center)
   {
      setBox(lx, ly, lz, center.getX(), center.getY(), center.getZ());
   }

   public void setBox(double lx, double ly, double lz, double centerX, double centerY, double centerZ)
   {
      boxSize.set(lx, ly, lz);
      boxCenter.set(centerX, centerY, centerZ);
   }

   public void setPlane(Point3D pointOnPlane, Vector3D planeNormal)
   {
      this.pointOnPlane.set(pointOnPlane);
      this.planeNormal.set(planeNormal);
   }

   private final Vector3D edgeVector = new Vector3D();
   private final Vector3D fromPlaneCenterToEdgeStart = new Vector3D();
   private final RecyclingArrayList<Point3D> unorderedIntersections = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3D.class));

   public List<Point3D> computeIntersections()
   {
      RecyclingArrayList<Point3D> intersections = new RecyclingArrayList<>(Point3D.class);
      computeIntersections(intersections);
      return intersections;
   }

   private final Point3D intersection = new Point3D();

   public void computeIntersections(RecyclingArrayList<Point3D> intersectionsToPack)
   {
      unorderedIntersections.clear();

      for (int i = 0; i < 12; i++)
      {
         Point3D edgeStart = boxEdges.get(i).getKey();
         Point3D edgeEnd = boxEdges.get(i).getValue();
         edgeVector.sub(edgeEnd, edgeStart);

         fromPlaneCenterToEdgeStart.sub(pointOnPlane, boxCenter);
         fromPlaneCenterToEdgeStart.setX(fromPlaneCenterToEdgeStart.getX() / boxSize.getX());
         fromPlaneCenterToEdgeStart.setY(fromPlaneCenterToEdgeStart.getY() / boxSize.getY());
         fromPlaneCenterToEdgeStart.setZ(fromPlaneCenterToEdgeStart.getZ() / boxSize.getZ());
         fromPlaneCenterToEdgeStart.sub(edgeStart);

         double dotNormalEdge = planeNormal.dot(edgeVector);

         if (Math.abs(dotNormalEdge) < 1.0e-5)
            continue;

         double scaleFactor = planeNormal.dot(fromPlaneCenterToEdgeStart) / dotNormalEdge;
         if (scaleFactor < 0.0 || scaleFactor > 1.0)
            continue;

         intersection.scaleAdd(scaleFactor, edgeVector, edgeStart);
         intersection.setX(intersection.getX() * boxSize.getX());
         intersection.setY(intersection.getY() * boxSize.getY());
         intersection.setZ(intersection.getZ() * boxSize.getZ());
         intersection.add(boxCenter);
         if (!listContains(unorderedIntersections, intersection))
            unorderedIntersections.add().set(intersection);

         if (unorderedIntersections.size() == 6) // That's the max number of possible intersections
            break;
      }
      reorderIntersections(unorderedIntersections, intersectionsToPack);
   }

   private final TDoubleArrayList orderedAngles = new TDoubleArrayList();
   private final Vector3D v0 = new Vector3D();
   private final Vector3D vi = new Vector3D();
   private final Vector3D vCross = new Vector3D();
   private final Point3D average = new Point3D();

   private void reorderIntersections(List<Point3D> unorderedIntersections, RecyclingArrayList<Point3D> intersectionsToPack)
   {
      intersectionsToPack.clear();
      if (unorderedIntersections.isEmpty())
         return;

      orderedAngles.reset();
      intersectionsToPack.add().set(unorderedIntersections.get(0));
      orderedAngles.add(0.0);

      average.set(0.0, 0.0, 0.0);
      for (int i = 0; i < unorderedIntersections.size(); i++)
         average.add(unorderedIntersections.get(i));
      average.scale(1.0 / unorderedIntersections.size());

      v0.sub(unorderedIntersections.get(0), average);
      v0.normalize();

      for (int i = 1; i < unorderedIntersections.size(); i++)
      {
         vi.sub(unorderedIntersections.get(i), average);
         vi.normalize();

         double angle = v0.dot(vi); // Gives [1.0, -1.0] for an angle in [0, Pi]

         vCross.cross(v0, vi);
         if (vCross.dot(planeNormal) < 0.0) // Allows to make a difference between the two angle ranges: [0, Pi] and [Pi, 2*Pi]
            angle = -2 - angle; // We're in the range [Pi, 2*Pi], this transform the dot original range [1, -1] to [-1, -3] where -3 is when vi & v0 point in the same direction. 
         // We obtained an "angle" that goes from 1 down to -3. It is transformed to be in the range [0, 4].
         angle -= 1.0;
         angle *= -1.0;

         // Binary search to figure out where the vertex should go in the list.
         int index = angleBinarySearch(orderedAngles, angle);
         if (index < 0)
            index = -index - 1;

         intersectionsToPack.insertAtIndex(index).set(unorderedIntersections.get(i));
         orderedAngles.insert(index, angle);
      }
   }

   private static int angleBinarySearch(TDoubleArrayList l, double key)
   {
      int low = 0;
      int high = l.size() - 1;

      while (low <= high)
      {
         int mid = (low + high) >>> 1;
         double midVal = l.get(mid);
         int cmp = compare(midVal, key);

         if (cmp < 0)
            low = mid + 1;
         else if (cmp > 0)
            high = mid - 1;
         else
            return mid; // key found
      }
      return -(low + 1); // key not found
   }

   private static int compare(double angle1, double angle2)
   {
      if (angle1 == angle2)
         return 0;
      else if (angle1 < angle2)
         return -1;
      // (angle1 > angle2)
      return 1;
   }

   private boolean listContains(List<Point3D> listOfPoints, Point3D pointToCheck)
   {
      for (int i = 0; i < listOfPoints.size(); i++)
      {
         if (epsilonEquals(listOfPoints.get(i), pointToCheck))
            return true;
      }
      return false;
   }

   private boolean epsilonEquals(Point3D point1, Point3D point2)
   {
      double diff;

      diff = point1.getX() - point2.getX();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > EPSILON * boxSize.getX())
         return false;

      diff = point1.getY() - point2.getY();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > EPSILON * boxSize.getY())
         return false;

      diff = point1.getZ() - point2.getZ();
      if (Double.isNaN(diff))
         return false;
      if ((diff < 0 ? -diff : diff) > EPSILON * boxSize.getZ())
         return false;

      return true;
   }
}
