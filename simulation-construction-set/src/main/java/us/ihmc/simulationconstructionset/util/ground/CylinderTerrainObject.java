package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.TransformTools;

public class CylinderTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   protected final BoundingBox3D boundingBox;
   protected final Cylinder3D cylinder;
   private final RigidBodyTransform location;
   private final double height;
   private final double radius;
   protected Graphics3DObject linkGraphics;

   private final Point3D tempPoint = new Point3D();
   private final Vector3D zVector = new Vector3D(0.0, 0.0, -1.0);

   // TODO: change box based surface equations to cylinder surface equations

   public CylinderTerrainObject(RigidBodyTransform location, double height, double radius, AppearanceDefinition appearance)
   {
      this.height = height;
      this.radius = radius;
      this.location = location;
      cylinder = new Cylinder3D(location, height, radius);

      Box3D box = new Box3D(location, radius * 2, radius * 2, height);
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
      RigidBodyTransform ret = new RigidBodyTransform(location);
      ret.appendTranslation(0.0, 0.0, -height / 2.0);
      return ret;
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
      Point3D testPoint = new Point3D(x, y, z);
      Line3D zLine = new Line3D(testPoint, zVector);

      Point3D intersection1 = new Point3D();
      Point3D intersection2 = new Point3D();
      int numberOfIntersections = cylinder.intersectionWith(zLine, intersection1, intersection2);

      if (numberOfIntersections == 0)
         return 0;
      else if (numberOfIntersections == 1)
         return intersection1.getZ();
      else
      {
         /*
          * TODO Review the following.
          * I think it should return only the highest of the two intersections which always
          * is intersection1.
          */
         if (testPoint.distanceSquared(intersection1) < testPoint.distanceSquared(intersection2))
            return intersection1.getZ();
         else
            return intersection2.getZ();
      }
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
      location.getRotationMatrix().getColumn(Axis.Z.ordinal(), axisDirection);
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
