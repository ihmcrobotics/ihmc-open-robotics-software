package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.Axis;


public class BalanceBeamTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final Point2D origin;
   private final BoundingBox3D boundingBox;
   private final Vector2D direction;
   private final double width;

   private final Vector2D tempVector = new Vector2D();
   private final Graphics3DObject linkGraphics;
   private final double heightAboveGround;

   public BalanceBeamTerrainObject(Point2D origin, double back, double forward, Vector2D direction, double width, double heightAboveGround, AppearanceDefinition appearance)
   {
      Point2D pForward = new Point2D(direction);
      pForward.scale(forward);

      Point2D pBack = new Point2D(direction);
      pBack.scale(-back);

      this.origin = new Point2D(origin);
      double xMin = Math.min(pForward.getX(), pBack.getX());
      double xMax = Math.max(pForward.getX(), pBack.getX());
      double yMin = Math.min(pForward.getY(), pBack.getY());
      double yMax = Math.max(pForward.getY(), pBack.getY());
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = zMin + heightAboveGround;
      
      Point3D minPoint = new Point3D(xMin, yMin, zMin);
      Point3D maxPoint = new Point3D(xMax, yMax, zMax);
      
      this.boundingBox = new BoundingBox3D(minPoint, maxPoint);
      
      this.direction = direction;
      this.width = width;
      this.heightAboveGround = heightAboveGround;

      linkGraphics = new Graphics3DObject();
      double height = width;
      linkGraphics.translate(origin.getX() + (xMin + xMax) / 2.0, origin.getY() + (yMin + yMax) / 2.0, heightAboveGround - height);
      double angle = Math.atan2(direction.getY(), direction.getX());
      linkGraphics.rotate(angle, Axis.Z);

      linkGraphics.addCube(forward + back, width, height, appearance);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = this.heightAt(x, y, z);
      this.surfaceNormalAt(x, y, z, normalToPack);
      
      return heightAt;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      double xFromOrigin = x - origin.getX();
      double yFromOrigin = y - origin.getY();
      
      tempVector.set(xFromOrigin, yFromOrigin);
      double componentAlongBalanceBeam = tempVector.dot(direction);
      tempVector.set(direction);
      tempVector.scale(componentAlongBalanceBeam);
      tempVector.setX(xFromOrigin - tempVector.getX());
      tempVector.setY(xFromOrigin - tempVector.getY());
      double distanceFromCenterOfBalanceBeamSquared = tempVector.lengthSquared();
      if (distanceFromCenterOfBalanceBeamSquared < width * width / 4.0)
      {
         return heightAboveGround;
      }
      else
      {
         return -1000.0;
      }
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }


   public void closestIntersectionTo(double x, double y, double z, Point3D intersection)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersection, Vector3D normal)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normal);
   }


   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      intersectionToPack.set(x, y, heightAt(x, y, z));
      surfaceNormalAt(x, y, z, normalToPack);
      
      return (z < intersectionToPack.getZ());
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInsideInclusive(x, y, z);
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
