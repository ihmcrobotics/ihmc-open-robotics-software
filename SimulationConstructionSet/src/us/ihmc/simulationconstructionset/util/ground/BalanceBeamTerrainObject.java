package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.BoundingBox3d;


public class BalanceBeamTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final Point2d origin;
   private final BoundingBox3d boundingBox;
   private final Vector2d direction;
   private final double width;

   private final Vector2d tempVector = new Vector2d();
   private final Graphics3DObject linkGraphics;
   private final double heightAboveGround;

   public BalanceBeamTerrainObject(Point2d origin, double back, double forward, Vector2d direction, double width, double heightAboveGround, AppearanceDefinition appearance)
   {
      Point2d pForward = new Point2d(direction);
      pForward.scale(forward);

      Point2d pBack = new Point2d(direction);
      pBack.scale(-back);

      this.origin = new Point2d(origin);
      double xMin = Math.min(pForward.getX(), pBack.getX());
      double xMax = Math.max(pForward.getX(), pBack.getX());
      double yMin = Math.min(pForward.getY(), pBack.getY());
      double yMax = Math.max(pForward.getY(), pBack.getY());
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = zMin + heightAboveGround;
      
      Point3d minPoint = new Point3d(xMin, yMin, zMin);
      Point3d maxPoint = new Point3d(xMax, yMax, zMax);
      
      this.boundingBox = new BoundingBox3d(minPoint, maxPoint);
      
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

   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = this.heightAt(x, y, z);
      this.surfaceNormalAt(x, y, z, normalToPack);
      
      return heightAt;
   }
   
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

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.set(0.0, 0.0, 1.0);
   }


   public void closestIntersectionTo(double x, double y, double z, Point3d intersection)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersection, Vector3d normal)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normal);
   }


   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      intersectionToPack.set(x, y, heightAt(x, y, z));
      surfaceNormalAt(x, y, z, normalToPack);
      
      return (z < intersectionToPack.getZ());
   }

   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInside(x, y, z);
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

   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

}
