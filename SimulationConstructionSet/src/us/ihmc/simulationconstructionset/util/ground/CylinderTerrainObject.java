package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.math3.geometry.euclidean.threed.Line;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.shapes.Cylinder3d;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;

public class CylinderTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   protected final BoundingBox3d boundingBox;
   protected final Cylinder3d cylinder;
   private final RigidBodyTransform location;
   private final double height;
   private final double radius;
   protected Graphics3DObject linkGraphics;

   private final Point3d tempPoint = new Point3d();
   private final Vector3d zVector = new Vector3d(0.0, 0.0, 1.0);


   // TODO: change box based surface equations to cylinder surface equations

   public CylinderTerrainObject(RigidBodyTransform location, double height, double radius, AppearanceDefinition appearance)
   {
      this.height = height;
      this.radius = radius;
      this.location = location;
      RigidBodyTransform bottomTransform = transformToBottomOfCylinder();
      cylinder = new Cylinder3d(bottomTransform, height, radius);

      Box3d box = new Box3d(location, radius * 2, radius * 2, height);
      Point3d[] vertices = box.getVertices();
      Point3d minPoint = new Point3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3d maxPoint = new Point3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (Point3d cornerPoint : vertices)
      {
         for (Direction direction : Direction.values())
         {
            double coordinate = MathTools.get(cornerPoint, direction);
            if (coordinate > MathTools.get(maxPoint, direction))
               MathTools.set(maxPoint, direction, coordinate);
            if (coordinate < MathTools.get(minPoint, direction))
               MathTools.set(minPoint, direction, coordinate);
         }
      }

//    minPoint.set(-1.0, -1.0, -1.0);
//    maxPoint.set(1.0, 1.0, 10.0);

      boundingBox = new BoundingBox3d(minPoint, maxPoint);

      addGraphics(appearance);
   }

   public CylinderTerrainObject(Vector3d center, double pitchDownDegrees, double yawDegrees, double height, double radius, AppearanceDefinition app)
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
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, 1e9);
      surfaceNormalAt(x, y, heightAt, normalToPack);

      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      Line axis = getAxis();

      Vector3D zero = new Vector3D(x, y, z);
      Vector3D zVector3D = new Vector3D(0.0, 0.0, 1.0);
      Line zLine = new Line(zero, zero.add(zVector3D));

      double distance = axis.distance(zLine);

      if (distance > radius)
         return 0.0;

      ArrayList<Point3d> intersections = new ArrayList<Point3d>();

      // Find intersections of line with planes at end of cylinder
      Point3d testPoint = new Point3d(x, y, z);
      Vector3d testDirection = zVector;

      addCylinderEndIntersectionsWithLine(intersections, testPoint, testDirection);

      if (intersections.size() < 2)
      {
         addCylinderSideIntersectionsWithLine(intersections, testPoint, testDirection);
      }

      if (intersections.isEmpty())
         return 0.0;

      // TODO: This returns the closest, not the highest, which is what heightAt should return...
      Point3d closestPoint = intersections.get(0);
      double closestSquaredDistance = closestPoint.distanceSquared(testPoint);
      for (Point3d intersection : intersections)
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

   private void addCylinderSideIntersectionsWithLine(ArrayList<Point3d> intersections, Point3d testPoint, Vector3d testDirection)
   {
      Point3d localTestPoint = new Point3d(testPoint);
      Vector3d localVector = new Vector3d(testDirection);
      transformLineToLocalCoordinates(localTestPoint, localVector);

      double a = localVector.getX() * localVector.getX() + localVector.getY() * localVector.getY();
      double b = 2 * (localTestPoint.getX() * localVector.getX() + localTestPoint.getY() * localVector.getY());
      double c = localTestPoint.getX() * localTestPoint.getX() + localTestPoint.getY() * localTestPoint.getY() - radius * radius;
      double[] t = solveQuadraticEquation(a, b, c);

      for (int i = 0; i < t.length; i++)
      {
         Point3d localSideIntersection = new Point3d();
         localSideIntersection.scaleAdd(t[i], localVector, localTestPoint);

         if ((localSideIntersection.getZ() >= -height / 2) && (localSideIntersection.getZ() <= height / 2))
         {
            location.transform(localSideIntersection);
            intersections.add(localSideIntersection);
         }
      }
   }

   private void addCylinderEndIntersectionsWithLine(ArrayList<Point3d> intersections, Point3d testPoint, Vector3d testDirection)
   {
      for (Cylinder3d.CylinderFaces face : Cylinder3d.CylinderFaces.values())
      {
         Plane3d plane = cylinder.getPlane(face);
         Point3d intersectionToPack = new Point3d();
         plane.getIntersectionWithLine(intersectionToPack, testPoint, testDirection);

         if (MathTools.isFinite(intersectionToPack) && (intersectionToPack.distanceSquared(plane.getPointCopy()) < radius * radius))
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

   private void transformLineToLocalCoordinates(Point3d localPointToPack, Vector3d localDirectionVectorToPack)
   {
      RigidBodyTransform inverseTransform = new RigidBodyTransform();
      inverseTransform.invert(location);
      inverseTransform.transform(localPointToPack);
      inverseTransform.transform(localDirectionVectorToPack);
   }

   public Line getAxis()
   {
      Vector3d axisOrigin = new Vector3d();
      location.getTranslation(axisOrigin);

      Vector3d axisDirection = getAxisDirectionCopy();

      Vector3D axisOrigin_ = vector3DFromVector3d(axisOrigin);
      Vector3D axisDirection_ = vector3DFromVector3d(axisDirection);
      Line line = new Line(axisOrigin_, axisOrigin_.add(axisDirection_));

      return line;
   }

   public Vector3d getAxisDirectionCopy()
   {
      Matrix3d rotation = new Matrix3d();
      location.getRotation(rotation);

      Vector3d axisDirection = new Vector3d();
      rotation.getColumn(Direction.Z.getIndex(), axisDirection);

      return axisDirection;
   }

   private Vector3D vector3DFromVector3d(Vector3d vector3d)
   {
      Vector3D vector3D = new Vector3D(vector3d.getX(), vector3d.getY(), vector3d.getZ());

      return vector3D;
   }

   public double getXMin()
   {
      return boundingBox.getXMin();
   }

   public double getYMin()
   {
      return boundingBox.getYMin();
   }

   public double getXMax()
   {
      return boundingBox.getXMax();
   }

   public double getYMax()
   {
      return boundingBox.getYMax();
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInside(x, y);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3d intersectionToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.checkIfInside(tempPoint, intersectionToPack, null);
   }

   private final Point3d ignoreIntesectionPoint = new Point3d();

   public void surfaceNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.checkIfInside(tempPoint, ignoreIntesectionPoint, normalToPack);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.checkIfInside(tempPoint, intersectionToPack, normalToPack);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
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
