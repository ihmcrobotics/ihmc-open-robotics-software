package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.shapes.Cylinder3d;
import us.ihmc.robotics.geometry.shapes.Plane3d;

public class CylinderTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private static final double EPS = 1.0e-12;

   protected final BoundingBox3D boundingBox;
   protected final Cylinder3d cylinder;
   private final RigidBodyTransform location;
   private final double height;
   private final double radius;
   protected Graphics3DObject linkGraphics;

   private final Point3D tempPoint = new Point3D();
   private final Vector3D zVector = new Vector3D(0.0, 0.0, 1.0);

   // TODO: change box based surface equations to cylinder surface equations

   public CylinderTerrainObject(RigidBodyTransform location, double height, double radius, AppearanceDefinition appearance)
   {
      this.height = height;
      this.radius = radius;
      this.location = location;
      RigidBodyTransform bottomTransform = transformToBottomOfCylinder();
      cylinder = new Cylinder3d(bottomTransform, height, radius);

      Box3d box = new Box3d(location, radius * 2, radius * 2, height);
      Point3D[] vertices = box.getVertices();
      Point3D minPoint = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3D maxPoint = new Point3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (Point3D cornerPoint : vertices)
      {
         for (int i = 0; i < 3; i++)
         {
            double coordinate = cornerPoint.getElement(i);
            if (coordinate > maxPoint.getElement(i))
               maxPoint.setElement(i, coordinate);
            if (coordinate < minPoint.getElement(i))
               minPoint.setElement(i, coordinate);
         }
      }

      boundingBox = new BoundingBox3D(minPoint, maxPoint);

      addGraphics(appearance);
   }

   public CylinderTerrainObject(Vector3D center, double pitchDownDegrees, double yawDegrees, double height, double radius, AppearanceDefinition app)
   {
      this(TransformTools.yawPitchDegreesTransform(center, yawDegrees, pitchDownDegrees), height, radius, app);
   }

   protected void addGraphics(AppearanceDefinition appearance)
   {
      RigidBodyTransform transform = transformToBottomOfCylinder();

      linkGraphics = new Graphics3DObject();
      linkGraphics.transform(transform);

      getLinkGraphics().addCylinder(height, radius, appearance);
   }

   private RigidBodyTransform transformToBottomOfCylinder()
   {
      return TransformTools.transformLocalZ(location, -height / 2.0);
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
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, 1e9);
      surfaceNormalAt(x, y, heightAt, normalToPack);

      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      Line3D axis = getAxis();
      Point3D testPoint = new Point3D(x, y, z);
      Line3D zLine = new Line3D(testPoint, zVector);

      double distance = axis.distance(zLine);

      if (distance > radius)
         return 0.0;

      ArrayList<Point3D> intersections = new ArrayList<Point3D>();

      // Find intersections of line with planes at end of cylinder
      Vector3D testDirection = zVector;

      addCylinderEndIntersectionsWithLine(intersections, testPoint, testDirection);

      if (intersections.size() < 2)
      {
         addCylinderSideIntersectionsWithLine(intersections, testPoint, testDirection);
      }

      if (intersections.isEmpty())
         return 0.0;

      // TODO: This returns the closest, not the highest, which is what heightAt should return...
      Point3D closestPoint = intersections.get(0);
      double closestSquaredDistance = closestPoint.distanceSquared(testPoint);
      for (Point3D intersection : intersections)
      {
         double nextSquaredDistance = intersection.distanceSquared(testPoint);
         if (nextSquaredDistance < closestSquaredDistance)
         {
            closestSquaredDistance = nextSquaredDistance;
            closestPoint = intersection;
         }
      }

      return closestPoint.getZ();

      // If not finite, then the line is perpendicular
      // Find abscissa on axis of closest point to line
      // If not in range (z0 to zh), no intersection
      // Else continue with finding intersections with side of cylinder
      // If both plane intersections d < r
      // Choose and return closest point of two (Takes care of parallel condition which would cause following to fail)
      // Find intersection(s) with side of cylinder and corresponding ptx, pty, ptz
      // in cylinder coordinates ptx^2+pty^2=r^2
      // If both ptz in zrange
      // Return closest point
      // If both ptz on same side of zrange
      // no answer
      // Should have one ptz in range, and one point with d<r
      // ->Return the point closest to x,y,z

   }

   private void addCylinderSideIntersectionsWithLine(ArrayList<Point3D> intersections, Point3D testPoint, Vector3D testDirection)
   {
      Point3D localTestPoint = new Point3D(testPoint);
      Vector3D localVector = new Vector3D(testDirection);
      transformLineToLocalCoordinates(localTestPoint, localVector);

      double a = localVector.getX() * localVector.getX() + localVector.getY() * localVector.getY();
      double b = 2 * (localTestPoint.getX() * localVector.getX() + localTestPoint.getY() * localVector.getY());
      double c = localTestPoint.getX() * localTestPoint.getX() + localTestPoint.getY() * localTestPoint.getY() - radius * radius;
      double[] t = solveQuadraticEquation(a, b, c);

      for (int i = 0; i < t.length; i++)
      {
         Point3D localSideIntersection = new Point3D();
         localSideIntersection.scaleAdd(t[i], localVector, localTestPoint);

         if ((localSideIntersection.getZ() > -height / 2 - EPS) && (localSideIntersection.getZ() < height / 2 + EPS))
         {
            location.transform(localSideIntersection);
            intersections.add(localSideIntersection);
         }
      }
   }

   private void addCylinderEndIntersectionsWithLine(ArrayList<Point3D> intersections, Point3D testPoint, Vector3D testDirection)
   {
      for (Cylinder3d.CylinderFaces face : Cylinder3d.CylinderFaces.values())
      {
         Plane3d plane = cylinder.getPlane(face);
         Point3D intersectionToPack = new Point3D();
         plane.getIntersectionWithLine(intersectionToPack, testPoint, testDirection);

         if (EuclidCoreMissingTools.isFinite(intersectionToPack) && (intersectionToPack.distanceSquared(plane.getPointCopy()) < radius * radius + EPS))
         {
            intersections.add(intersectionToPack);
         }
      }
   }

   private double[] solveQuadraticEquation(double ax2, double bx, double c)
   {
      double val = Math.sqrt(bx * bx - 4 * ax2 * c);
      double[] x = {(-bx + val) / (2 * ax2), (-bx - val) / (2 * ax2)};

      return x;
   }

   private void transformLineToLocalCoordinates(Point3D localPointToPack, Vector3D localDirectionVectorToPack)
   {
      RigidBodyTransform inverseTransform = new RigidBodyTransform();
      inverseTransform.setAndInvert(location);
      inverseTransform.transform(localPointToPack);
      inverseTransform.transform(localDirectionVectorToPack);
   }

   public Line3D getAxis()
   {
      Point3D axisOrigin = new Point3D();
      location.getTranslation(axisOrigin);

      Vector3D axisDirection = getAxisDirectionCopy();

      return new Line3D(axisOrigin, axisDirection);
   }

   public Vector3D getAxisDirectionCopy()
   {
      Vector3D axisDirection = new Vector3D();
      location.getRotationMatrix().getColumn(Direction.Z.getIndex(), axisDirection);
      return axisDirection;
   }

   public double getXMin()
   {
      return boundingBox.getMinX();
   }

   public double getYMin()
   {
      return boundingBox.getMinY();
   }

   public double getXMax()
   {
      return boundingBox.getMaxX();
   }

   public double getYMax()
   {
      return boundingBox.getMaxY();
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInsideInclusive(x, y);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3D intersectionToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.checkIfInside(tempPoint, intersectionToPack, null);
   }

   private final Point3D ignoreIntesectionPoint = new Point3D();

   public void surfaceNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.checkIfInside(tempPoint, ignoreIntesectionPoint, normalToPack);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.checkIfInside(tempPoint, intersectionToPack, normalToPack);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      return cylinder.checkIfInside(tempPoint, intersectionToPack, normalToPack);
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
