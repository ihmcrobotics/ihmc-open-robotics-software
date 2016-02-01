package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class Box3d implements Shape3d
{
   public static final int NUM_VERTICES = 8;
   public static final int NUM_SIDES = 6;
   public static final int NUM_VERTICES_PER_FACE = 4;

   protected final RigidBodyTransform transform = new RigidBodyTransform();
   private final EnumMap<Direction, Double> dimensions = new EnumMap<Direction, Double>(Direction.class);
   private boolean dirtyBit = true;

   private final EnumMap<FaceName, Plane3d> faces = new EnumMap<FaceName, Plane3d>(FaceName.class);

   // These are temporary storage used to prevent garbage collection in a few of the methods.
   // Box3d is low level enough that it should never generate trash unless you call one of the
   // methods ending in Copy.
   private final Point3d temporaryPoint = new Point3d();
   private final Vector3d temporaryVector = new Vector3d();
   private final Matrix3d temporaryMatrix = new Matrix3d();

   public Box3d()
   {
      for (FaceName faceName : FaceName.values())
      {
         faces.put(faceName, new Plane3d());
      }

      transform.setIdentity();
      setDimensions(1.0, 1.0, 1.0);
      this.dirtyBit = true;
   }

   public Box3d(RigidBodyTransform configuration, double lengthX, double widthY, double heightZ)
   {
      for (FaceName faceName : FaceName.values())
      {
         faces.put(faceName, new Plane3d());
      }

      setTransform(configuration);
      setDimensions(lengthX, widthY, heightZ);
      this.dirtyBit = true;
   }

   public Box3d(Box3d other)
   {
      this();
      this.set(other);
   }

   public Box3d(double lengthX, double widthY, double heightZ)
   {
      this();
      setDimensions(lengthX, widthY, heightZ);
   }

   public Box3d(RigidBodyTransform configuration, double[] dimensions)
   {
      this(configuration, dimensions[0], dimensions[1], dimensions[2]);
   }

   public void getCenter(Tuple3d centerToPack)
   {
      transform.get(temporaryVector);
      centerToPack.set(temporaryVector);
   }

   public Point3d getCenterCopy()
   {
      Point3d ret = new Point3d();
      getCenter(ret);

      return ret;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(this.transform);
   }

   public RigidBodyTransform getTransformCopy()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransform(ret);

      return ret;
   }

   public void getRotation(Matrix3d rotationMatrixToPack)
   {
      this.transform.get(rotationMatrixToPack);
   }

   public Matrix3d getRotationCopy()
   {
      Matrix3d ret = new Matrix3d();
      getRotation(ret);

      return ret;
   }

   public double getDimension(Direction direction)
   {
      return dimensions.get(direction);
   }

   public double getLength()
   {
      return getDimension(Direction.X);
   }

   public double getWidth()
   {
      return getDimension(Direction.Y);
   }

   public double getHeight()
   {
      return getDimension(Direction.Z);
   }

   public Plane3d getFace(FaceName faceName)
   {
      updateFacesIfNecessary();

      return faces.get(faceName);
   }

   public synchronized void set(Box3d other)
   {
      setTransform(other.transform);
      setDimensions(other.dimensions);
      this.dirtyBit = other.dirtyBit;

      //    if (this.dirtyBit)
      //    {
      //       for (FaceName faceName : FaceName.values())
      //       {
      //          this.faces.get(faceName).set(other.faces.get(faceName));
      //       }
      //    }
   }

   public void setDimensions(EnumMap<Direction, Double> dimensions)
   {
      this.dimensions.putAll(dimensions);
      this.dirtyBit = true;
   }

   public void setDimensions(double lengthX, double widthY, double heightZ)
   {
      this.dimensions.put(Direction.X, lengthX);
      this.dimensions.put(Direction.Y, widthY);
      this.dimensions.put(Direction.Z, heightZ);
      this.dirtyBit = true;
   }

   public void scale(double scale)
   {
      for (Direction direction : Direction.values())
      {
         dimensions.put(direction, dimensions.get(direction) * scale);
      }

      this.dirtyBit = true;
   }

   public double distance(Point3d point)
   {
      updateFacesIfNecessary();
      temporaryPoint.set(point);
      orthogonalProjection(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   public void orthogonalProjection(Point3d point)
   {
      updateFacesIfNecessary();

      for (Plane3d face : faces.values())
      {
         if (face.isOnOrAbove(point))
         {
            face.orthogonalProjection(point);
         }
      }
   }

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void applyTransform(RigidBodyTransform transform)
   {
      tempTransform.set(transform);
      tempTransform.multiply(this.transform);
      this.transform.set(tempTransform);
      this.dirtyBit = true;
   }

   public void setTransform(RigidBodyTransform transform)
   {
      this.transform.set(transform);
      this.dirtyBit = true;
   }

   public void setRotation(Matrix3d rotation)
   {
      this.transform.setRotation(rotation);
      this.dirtyBit = true;
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, temporaryMatrix);
      setRotation(temporaryMatrix);
   }

   public void setTranslation(Tuple3d translation)
   {
      this.temporaryVector.set(translation);
      this.transform.setTranslationAndIdentityRotation(temporaryVector);
      this.dirtyBit = true;
   }

   public boolean isInsideOrOnSurface(Point3d point)
   {
      return (isInsideOrOnSurface(point, 0.0));
   }

   public boolean isInsideOrOnSurface(Point3d point, double epsilon)
   {
      updateFacesIfNecessary();

      for (FaceName faceName : FaceName.values)
      {
         Plane3d face = faces.get(faceName);
         if (!face.isOnOrBelow(point, epsilon))
         {
            return false;
         }
      }

      return true;
   }

   public boolean checkIfInside(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      updateFacesIfNecessary();

      if (isInsideOrOnSurface(pointInWorldToCheck))
      {
         Plane3d nearestFace = getClosestFace(pointInWorldToCheck);

         if (closestPointToPack != null)
         {
            closestPointToPack.set(pointInWorldToCheck);
            nearestFace.orthogonalProjection(closestPointToPack);
         }

         if (normalToPack != null)
         {
            nearestFace.getNormal(normalToPack);
         }

         return true;
      }
      else
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(pointInWorldToCheck);
            this.orthogonalProjection(closestPointToPack);
         }

         if (normalToPack != null)
         {
            if (closestPointToPack.distance(pointInWorldToCheck) == 0.0)
            {
               Plane3d nearestFace = getClosestFace(closestPointToPack);
               nearestFace.getNormal(normalToPack);
            }
            else
            {
               normalToPack.set(pointInWorldToCheck);
               normalToPack.sub(closestPointToPack);
               normalToPack.normalize();
            }
         }

         return false;
      }
   }

   public Plane3d getClosestFace(Point3d point)
   {
      updateFacesIfNecessary();

      double nearestDistance = Double.POSITIVE_INFINITY;
      Plane3d nearestFace = null;

      for (FaceName faceName : FaceName.values)
      {
         Plane3d face = faces.get(faceName);
         double distance = face.distance(point);
         if (distance < nearestDistance)
         {
            nearestDistance = distance;
            nearestFace = face;
         }
      }
      return nearestFace;
   }

   public Point3d[] getVertices()
   {
      Point3d[] ret = new Point3d[NUM_VERTICES];
      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = new Point3d();
      }
      computeVertices(ret);
      return ret;
   }

   public void computeVertices(Point3d[] verticesToPack)
   {
      MathTools.checkIfEqual(NUM_VERTICES, verticesToPack.length);

      List<Vector3d> signPermutations = new ArrayList<Vector3d>(NUM_VERTICES);
      int[] signs = new int[] { -1, 1 };
      for (int xSign : signs)
      {
         for (int ySign : signs)
         {
            for (int zSign : signs)
               signPermutations.add(new Vector3d(xSign, ySign, zSign));
         }
      }

      computeVertices(verticesToPack, signPermutations);
   }

   public void computeVertices(Point3d[] verticesToPack, FaceName faceName)
   {
      MathTools.checkIfEqual(NUM_VERTICES_PER_FACE, verticesToPack.length);

      Direction faceDirection = faceName.getDirection();
      int faceSign = faceName.sign();

      Direction[] directionValues = Direction.values();
      Direction[] otherDirections = new Direction[directionValues.length - 1];
      int index = 0;
      for (Direction direction : directionValues)
      {
         if (direction != faceDirection)
            otherDirections[index++] = direction;
      }

      int[] signs = new int[] { -1, 1 };
      List<Vector3d> signPermutations = new ArrayList<Vector3d>(NUM_VERTICES_PER_FACE);
      for (int sign1 : signs)
      {
         for (int sign2Switch : signs)
         {
            Vector3d signPermutation = new Vector3d();
            MathTools.set(signPermutation, otherDirections[0], sign1);
            MathTools.set(signPermutation, otherDirections[1], sign1 * sign2Switch);
            MathTools.set(signPermutation, faceDirection, faceSign);
            signPermutations.add(signPermutation);
         }
      }
      computeVertices(verticesToPack, signPermutations);
   }

   private void computeVertices(Point3d[] verticesToPack, List<Vector3d> signPermutations)
   {
      double xHalfLength = getDimension(Direction.X) / 2.0;
      double yHalfWidth = getDimension(Direction.Y) / 2.0;
      double zHalfHeight = getDimension(Direction.Z) / 2.0;

      for (int i = 0; i < signPermutations.size(); i++)
      {
         Vector3d signPermutation = signPermutations.get(i);
         verticesToPack[i].set(signPermutation.getX() * xHalfLength, signPermutation.getY() * yHalfWidth, signPermutation.getZ() * zHalfHeight);
         transform.transform(verticesToPack[i]);
      }
   }

   // /CLOVER:OFF
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      transform.get(temporaryVector);
      builder.append("center: (" + temporaryVector.getX() + ", " + temporaryVector.getY() + ", " + temporaryVector.getZ() + ")\n");
      builder.append("dimensions: (" + dimensions.get(Direction.X) + ", " + dimensions.get(Direction.Y) + ", " + dimensions.get(Direction.Z) + ")\n");
      builder.append("faces: \n");

      updateFacesIfNecessary();

      for (Plane3d face : faces.values())
      {
         builder.append(face.toString());
      }

      return builder.toString();
   }

   // /CLOVER:ON

   private void updateFacesIfNecessary()
   {
      if (dirtyBit)
      {
         transform.get(temporaryMatrix);

         for (FaceName faceName : faces.keySet())
         {
            Plane3d face = faces.get(faceName);

            temporaryMatrix.getColumn(faceName.getDirection().getIndex(), temporaryVector);
            temporaryVector.scale(faceName.sign());
            face.setNormal(temporaryVector);

            for (Direction direction : Direction.values())
            {
               double coordinate = (direction == faceName.getDirection()) ? faceName.sign() * getDimension(direction) / 2.0 : 0.0;
               MathTools.set(temporaryPoint, direction, coordinate);
            }

            transform.transform(temporaryPoint);
            face.setPoint(temporaryPoint);
         }
      }

      dirtyBit = false;
   }

   public static enum FaceName
   {
      PLUSX(true, Direction.X), PLUSY(true, Direction.Y), PLUSZ(true, Direction.Z), MINUSX(false, Direction.X), MINUSY(false, Direction.Y), MINUSZ(false,
            Direction.Z);

      private final boolean positive;
      private final Direction direction;
      public static final FaceName[] values = values();

      private FaceName(boolean positive, Direction direction)
      {
         this.positive = positive;
         this.direction = direction;
      }

      public int sign()
      {
         return positive ? 1 : -1;
      }

      public Direction getDirection()
      {
         return direction;
      }

      public static FaceName get(boolean positive, Direction direction)
      {
         if (direction == null)
            throw new RuntimeException("direction == null");

         for (FaceName faceName : FaceName.values())
         {
            if ((faceName.positive == positive) && (faceName.direction == direction))
               return faceName;
         }

         throw new RuntimeException("should never get here");
      }
   }

}
