package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

/**
 * Ramp where the center of ramp start side is origin. 0 to X -Y/2 +Y/2 0 to Z
 */
public class Ramp3d extends Shape3D<Ramp3d>
{
   private final Size3d size = new Size3d();

   private double rampLength;
   private double angleOfRampIncline;

   public Ramp3d(Ramp3d ramp3d)
   {
      setPose(ramp3d);
      setSize(ramp3d.getLength(), ramp3d.getWidth(), ramp3d.getHeight());
   }

   public Ramp3d(double width, double length, double height)
   {
      setSize(length, width, height);
   }

   public Ramp3d(RigidBodyTransform transform, double width, double length, double height)
   {
      setPose(transform);
      setSize(length, width, height);
   }

   public Ramp3d(Pose3D pose, double width, double length, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   @Override
   public void set(Ramp3d ramp3d)
   {
      if (this != ramp3d)
      {
         setPose(ramp3d);
         setSize(ramp3d.size);
      }
   }

   public double getWidth()
   {
      return size.getWidth();
   }

   public void setWidth(double width)
   {
      size.setWidth(width);
   }

   public double getHeight()
   {
      return size.getHeight();
   }

   public void setHeight(double height)
   {
      size.setHeight(height);
      updateRamp();
   }

   public double getLength()
   {
      return size.getLength();
   }

   public void setLength(double length)
   {
      size.setLength(length);
      updateRamp();
   }

   public void setSize(Tuple3DReadOnly size)
   {
      setSize(size.getX(), size.getY(), size.getZ());
   }

   public void setSize(double length, double width, double height)
   {
      size.setLengthWidthHeight(length, width, height);
      updateRamp();
   }

   private void updateRamp()
   {
      rampLength = Math.sqrt(EuclidCoreTools.normSquared(size.getLength(), size.getHeight()));
      angleOfRampIncline = Math.atan(size.getHeight() / size.getLength());
   }

   public double getRampLength()
   {
      return rampLength;
   }

   public void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(-size.getHeight() / rampLength, 0.0, size.getLength() / rampLength);
      transformToWorld(surfaceNormalToPack);
   }

   public double getRampIncline()
   {
      return angleOfRampIncline;
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double rampDirectionX = size.getLength() / rampLength;
      double rampDirectionZ = size.getHeight() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double halfWidth = 0.5 * size.getWidth();
      if (z < 0.0)
      { // Query is below the ramp
         double xClosest = MathTools.clamp(x, 0.0, size.getLength());
         double yClosest = MathTools.clamp(y, -0.5 * size.getWidth(), halfWidth);
         double zClosest = 0.0;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         if (xClosest == x && yClosest == y)
         {
            if (normalToPack != null)
            {
               normalToPack.set(0.0, 0.0, -1.0);
            }

            return -z;
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (x > size.getLength() || EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, size.getLength(), size.getHeight(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = size.getLength();
         double yClosest = MathTools.clamp(y, -0.5 * size.getWidth(), halfWidth);
         double zClosest = MathTools.clamp(z, 0.0, size.getHeight());

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         if (yClosest == y && zClosest == z)
         {
            if (normalToPack != null)
            {
               normalToPack.set(1.0, 0.0, 0.0);
            }

            return x - size.getLength();
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = MathTools.clamp(y, -0.5 * size.getWidth(), halfWidth);
         double zClosest = 0.0;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
      }
      else if (Math.abs(y) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, y);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = x;
            double zClosest = z;

            if (closestPointToPack != null)
            {
               closestPointToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, Math.copySign(1.0, y), 0.0);
            }

            return Math.abs(y) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = x * rampDirectionX + z * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            if (closestPointToPack != null)
            {
               closestPointToPack.set(xClosest, yClosest, zClosest);
            }

            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = x * rampDirectionX + z * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = y;
         double zClosest = dot * rampDirectionZ;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClosest, yClosest, zClosest);
         }

         if (normalToPack != null)
         {
            normalToPack.set(rampNormalX, 0.0, rampNormalZ);
         }

         return rampDirectionX * z - x * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - y);
         double distanceToLeftFace = halfWidth - y;
         double distanceToRearFace = size.getLength() - x;
         double distanceToBottomFace = z;
         double distanceToSlopeFace = -(rampDirectionX * z - x * rampDirectionZ);

         if (isFirstValueMinimum(distanceToRightFace, distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the right face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(x, -halfWidth, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, -1.0, 0.0);
            }

            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(x, halfWidth, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, 1.0, 0.0);
            }

            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(size.getLength(), y, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(1.0, 0.0, 0.0);
            }

            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            if (closestPointToPack != null)
            {
               closestPointToPack.set(x, y, 0.0);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, 0.0, -1.0);
            }

            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            if (closestPointToPack != null)
            {
               double dot = x * rampDirectionX + z * rampDirectionZ;
               double xClosest = dot * rampDirectionX;
               double yClosest = y;
               double zClosest = dot * rampDirectionZ;

               closestPointToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(rampNormalX, 0.0, rampNormalZ);
            }

            return -distanceToSlopeFace;
         }
      }
   }

   private double computeNormalAndDistanceFromClosestPoint(double x, double y, double z, double xClosest, double yClosest, double zClosest,
                                                           Vector3DBasics normalToPack)
   {
      double dx = x - xClosest;
      double dy = y - yClosest;
      double dz = z - zClosest;

      double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

      if (normalToPack != null)
      {
         normalToPack.set(dx, dy, dz);
         normalToPack.scale(1.0 / distance);
      }

      return distance;
   }

   private boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3, double value4)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3 && possibleMin <= value4;
   }

   private boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3;
   }

   private boolean isFirstValueMinimum(double possibleMin, double value1, double value2)
   {
      return possibleMin <= value1 && possibleMin <= value2;
   }

   @Override
   protected boolean isInsideEpsilonShapeFrame(double x, double y, double z, double epsilon)
   {
      if (z < -epsilon)
         return false;

      if (x > size.getLength() + epsilon)
         return false;

      double halfWidth = 0.5 * size.getWidth() + epsilon;

      if (y < -halfWidth || y > halfWidth)
         return false;

      double rampDirectionX = size.getLength() / rampLength;
      double rampDirectionZ = size.getHeight() / rampLength;

      // Computing the signed distance between the query and the slope face, negative value means the query is below the slope.
      if (rampDirectionX * z - x * rampDirectionZ > epsilon)
         return false;

      return true;
   }

   @Override
   public boolean epsilonEquals(Ramp3d other, double epsilon)
   {
      return size.epsilonEquals(other.size, epsilon);
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      size.setToZero();
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      size.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || size.containsNaN();
   }

   @Override
   public String toString()
   {
      return "size = " + size + ", + transform = " + getPoseString() + "\n";
   }
}
