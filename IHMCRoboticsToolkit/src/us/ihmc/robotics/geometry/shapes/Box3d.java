package us.ihmc.robotics.geometry.shapes;

import java.util.EnumMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;

/**
 * Box where base frame is in the center.
 */
public class Box3d extends Shape3d<Box3d>
{
   public static final int NUM_VERTICES = 8;
   public static final int NUM_SIDES = 6;
   public static final int NUM_VERTICES_PER_FACE = 4;

   private final EnumMap<Direction, Double> dimensions;
   private final EnumMap<FaceName, Plane3d> faces;
   private boolean facesAreOutOfDate;

   private final Point3d temporaryPoint;
   private final Matrix3d temporaryMatrix;

   public Box3d()
   {
      this(1.0, 1.0, 1.0);
   }

   public Box3d(Box3d other)
   {
      this(other.getTransformFromShapeFrameUnsafe(), other.getLength(), other.getWidth(), other.getHeight());
   }

   public Box3d(RigidBodyTransform configuration, double[] dimensions)
   {
      this(configuration, dimensions[0], dimensions[1], dimensions[2]);
   }

   public Box3d(double lengthX, double widthY, double heightZ)
   {
      dimensions = new EnumMap<Direction, Double>(Direction.class);
      faces = new EnumMap<FaceName, Plane3d>(FaceName.class);
      temporaryPoint = new Point3d();
      temporaryMatrix = new Matrix3d();

      commonConstructor(lengthX, widthY, heightZ);
   }

   public Box3d(RigidBodyTransform transform, double length, double width, double height)
   {
      setTransform(transform);
      dimensions = new EnumMap<Direction, Double>(Direction.class);
      faces = new EnumMap<FaceName, Plane3d>(FaceName.class);
      temporaryPoint = new Point3d();
      temporaryMatrix = new Matrix3d();

      commonConstructor(length, width, height);
   }

   public Box3d(Point3d position, Quat4d orientation, double length, double width, double height)
   {
      setPose(position, orientation);
      dimensions = new EnumMap<Direction, Double>(Direction.class);
      faces = new EnumMap<FaceName, Plane3d>(FaceName.class);
      temporaryPoint = new Point3d();
      temporaryMatrix = new Matrix3d();

      commonConstructor(length, width, height);
   }

   public void commonConstructor(double length, double width, double height)
   {
      for (FaceName faceName : FaceName.values())
      {
         faces.put(faceName, new Plane3d());
      }

      setDimensions(length, width, height);
      facesAreOutOfDate = true;
   }

   public void getCenter(Point3d centerToPack)
   {
      getPosition(centerToPack);
   }

   public double getDimension(Direction direction)
   {
      return dimensions.get(direction);
   }
   
   public double getSizeX()
   {
      return dimensions.get(Direction.X);
   }

   public double getSizeY()
   {
      return dimensions.get(Direction.Y);
   }

   public double getSizeZ()
   {
      return dimensions.get(Direction.Z);
   }

   public double getLength()
   {
      return dimensions.get(Direction.X);
   }

   public double getWidth()
   {
      return dimensions.get(Direction.Y);
   }

   public double getHeight()
   {
      return dimensions.get(Direction.Z);
   }

   public void getFace(Direction direction, boolean positive, Plane3d planeToPack)
   {
      planeToPack.set(faces.get(FaceName.get(positive, direction)));
      transformFromShapeFrame(planeToPack);
   }
   
   @Override
   public void setToZero()
   {
      setDimensions(0.0, 0.0, 0.0);
   }

   @Override
   public void setToNaN()
   {
      setDimensions(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(getHeight()) || Double.isNaN(getLength()) || Double.isNaN(getWidth());
   }

   @Override
   public boolean epsilonEquals(Box3d other, double epsilon)
   {
      return MathTools.epsilonEquals(getWidth(), other.getWidth(), epsilon) && MathTools.epsilonEquals(getLength(), other.getLength(), epsilon)
            && MathTools.epsilonEquals(getHeight(), other.getHeight(), epsilon);
   }

   @Override
   public synchronized void set(Box3d other)
   {
      if (other != this)
      {
         setTransformFromShapeFrame(other.getTransformFromShapeFrameUnsafe());
         setDimensions(other.dimensions);
         facesAreOutOfDate = true;
      }
   }

   public void setDimensions(EnumMap<Direction, Double> dimensions)
   {
      this.dimensions.putAll(dimensions);
      facesAreOutOfDate = true;
   }

   public void setDimensions(double lengthX, double widthY, double heightZ)
   {
      dimensions.put(Direction.X, lengthX);
      dimensions.put(Direction.Y, widthY);
      dimensions.put(Direction.Z, heightZ);
      facesAreOutOfDate = true;
   }

   public void setFromTransform(RigidBodyTransform transform)
   {
      setTransformFromShapeFrame(transform);
      facesAreOutOfDate = true;
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, temporaryMatrix);
      setRotation(temporaryMatrix);
   }

   @Override
   public void setOrientation(Matrix3d rotation)
   {
      super.setOrientation(rotation);
      facesAreOutOfDate = true;
   }

   @Override
   public void setPosition(Point3d translation)
   {
      super.setPosition(translation);
      facesAreOutOfDate = true;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      super.applyTransform(transform);
      facesAreOutOfDate = true;
   }

   public void scale(double scale)
   {
      for (Direction direction : Direction.values())
      {
         dimensions.put(direction, dimensions.get(direction) * scale);
      }

      this.facesAreOutOfDate = true;
   }

   @Override
   protected double distanceShapeFrame(Point3d point)
   {
      ensureFacesAreUpToDate();
      temporaryPoint.set(point);
      orthogonalProjectionShapeFrame(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3d point)
   {
      ensureFacesAreUpToDate();

      for (Plane3d face : faces.values())
      {
         if (face.isOnOrAbove(point))
         {
            face.orthogonalProjection(point);
         }
      }
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3d point, double epsilon)
   {
      ensureFacesAreUpToDate();

      boolean isInsideOrOnSurface = true;
      for (FaceName faceName : FaceName.values)
      {
         Plane3d face = faces.get(faceName);
         if (!face.isOnOrBelow(point, epsilon))
         {
            isInsideOrOnSurface = false;
            break;
         }
      }
      
      return isInsideOrOnSurface;
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      ensureFacesAreUpToDate();

      if (closestPointToPack == null)
         closestPointToPack = new Point3d();

      if (isInsideOrOnSurfaceShapeFrame(pointInWorldToCheck, 0.0))
      {
         Plane3d nearestFace = getClosestFace(pointInWorldToCheck);

         closestPointToPack.set(pointInWorldToCheck);
         nearestFace.orthogonalProjection(closestPointToPack);

         if (normalToPack != null)
         {
            nearestFace.getNormal(normalToPack);
         }

         return true;
      }
      else
      {
         closestPointToPack.set(pointInWorldToCheck);
         orthogonalProjectionShapeFrame(closestPointToPack);

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

   private Plane3d getClosestFace(Point3d point)
   {
      ensureFacesAreUpToDate();

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

   /**
    * Not real-time
    */
   public Point3d[] getVertices()
   {
      Point3d[] vertices = new Point3d[NUM_VERTICES];
      vertices[0] = new Point3d();
      vertices[1] = new Point3d();
      vertices[2] = new Point3d();
      vertices[3] = new Point3d();
      vertices[4] = new Point3d();
      vertices[5] = new Point3d();
      vertices[6] = new Point3d();
      vertices[7] = new Point3d();
      computeVerticesShapeFrame(vertices);
      transformVerticesFromShapeFrame(vertices);
      return vertices;
   }

   public void computeVertices(Point3d[] verticesToPack)
   {
      computeVerticesShapeFrame(verticesToPack);
      transformVerticesFromShapeFrame(verticesToPack);
   }
   
   private void computeVerticesShapeFrame(Point3d[] verticesToPack)
   {
      MathTools.checkIfEqual(NUM_VERTICES, verticesToPack.length);
      
      double dx = dimensions.get(Direction.X) / 2.0;
      double dy = dimensions.get(Direction.Y) / 2.0;
      double dz = dimensions.get(Direction.Z) / 2.0;
      
      verticesToPack[0].set(-dx, -dy, -dz);
      verticesToPack[1].set(-dx, -dy, +dz);
      verticesToPack[2].set(-dx, +dy, -dz);
      verticesToPack[3].set(-dx, +dy, +dz);
      verticesToPack[4].set(+dx, -dy, -dz);
      verticesToPack[5].set(+dx, -dy, +dz);
      verticesToPack[6].set(+dx, +dy, -dz);
      verticesToPack[7].set(+dx, +dy, +dz);
   }

   public void computeVertices(Point3d[] verticesToPack, FaceName faceName)
   {
      computeVerticesShapeFrame(verticesToPack, faceName);
      transformVerticesFromShapeFrame(verticesToPack);
   }
   
   private void computeVerticesShapeFrame(Point3d[] verticesToPack, FaceName faceName)
   {
      MathTools.checkIfEqual(NUM_VERTICES_PER_FACE, verticesToPack.length);
      
      double dx = dimensions.get(Direction.X) / 2.0;
      double dy = dimensions.get(Direction.Y) / 2.0;
      double dz = dimensions.get(Direction.Z) / 2.0;
      
      if (faceName.getDirection() == Direction.X)
      {
         verticesToPack[0].set(faceName.sign() * dx, -dy, -dz);
         verticesToPack[1].set(faceName.sign() * dx, -dy, +dz);
         verticesToPack[2].set(faceName.sign() * dx, +dy, -dz);
         verticesToPack[3].set(faceName.sign() * dx, +dy, +dz);
      }
      else if (faceName.getDirection() == Direction.Y)
      {
         verticesToPack[0].set(-dx, faceName.sign() * dy, -dz);
         verticesToPack[1].set(-dx, faceName.sign() * dy, +dz);
         verticesToPack[2].set(+dx, faceName.sign() * dy, -dz);
         verticesToPack[3].set(+dx, faceName.sign() * dy, +dz);
      }
      else if (faceName.getDirection() == Direction.Z)
      {
         verticesToPack[0].set(-dx, -dy, faceName.sign() * dz);
         verticesToPack[1].set(+dx, -dy, faceName.sign() * dz);
         verticesToPack[2].set(-dx, +dy, faceName.sign() * dz);
         verticesToPack[3].set(+dx, +dy, faceName.sign() * dz);
      }
   }
   
   private void transformVerticesFromShapeFrame(Point3d[] verticesInShapeFrame)
   {
      for (int i = 0; i < verticesInShapeFrame.length; i++)
      {
         transformFromShapeFrame(verticesInShapeFrame[i]);
      }
   }

   private void ensureFacesAreUpToDate()
   {
      if (facesAreOutOfDate)
      {
         for (FaceName faceName : faces.keySet())
         {
            Plane3d face = faces.get(faceName);
            
            double xNormal = faceName.getDirection() == Direction.X ? faceName.sign() * 1.0 : 0.0;
            double yNormal = faceName.getDirection() == Direction.Y ? faceName.sign() * 1.0 : 0.0;
            double zNormal = faceName.getDirection() == Direction.Z ? faceName.sign() * 1.0 : 0.0;
            
            face.setNormal(xNormal, yNormal, zNormal);

            double xPoint = faceName.getDirection() == Direction.X ? faceName.sign() * dimensions.get(Direction.X) / 2.0 : 0.0;
            double yPoint = faceName.getDirection() == Direction.Y ? faceName.sign() * dimensions.get(Direction.Y) / 2.0 : 0.0;
            double zPoint = faceName.getDirection() == Direction.Z ? faceName.sign() * dimensions.get(Direction.Z) / 2.0 : 0.0;
            
            face.setPoint(xPoint, yPoint, zPoint);
         }
      }

      facesAreOutOfDate = false;
   }

   public static enum FaceName
   {
      PLUSX(true, Direction.X),
      PLUSY(true, Direction.Y),
      PLUSZ(true, Direction.Z),
      MINUSX(false, Direction.X),
      MINUSY(false, Direction.Y),
      MINUSZ(false, Direction.Z);

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

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("dimensions: (" + dimensions.get(Direction.X) + ", " + dimensions.get(Direction.Y) + ", " + dimensions.get(Direction.Z) + ")\n");
      builder.append("faces: \n");

      ensureFacesAreUpToDate();

      for (Plane3d face : faces.values())
      {
         builder.append(face.toString());
      }

      return builder.toString();
   }

   /**
    * @deprecated Use getCenter(Point3d)
    */
   public Point3d getCenterCopy()
   {
      Point3d ret = new Point3d();
      getCenter(ret);
      return ret;
   }

   /**
    * @deprecated Use getTransform(RigidBodyTransform)
    */
   public RigidBodyTransform getTransformCopy()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransform(ret);
      return ret;
   }

   /**
    * @deprecated Use getOrientation(Matrix3d) 
    */
   public void getRotation(Matrix3d rotationMatrixToPack)
   {
      getOrientation(rotationMatrixToPack);
   }

   /**
    * @deprecated Use getOrientation(Matrix3d) 
    */
   public Matrix3d getRotationCopy()
   {
      Matrix3d ret = new Matrix3d();
      getRotation(ret);
      return ret;
   }

   /**
    * @deprecated Makes garbage. Use getFace(Direction, boolean, Plane3d)
    */
   public Plane3d getFace(FaceName faceName)
   {
      ensureFacesAreUpToDate();
   
      Plane3d facePlane = new Plane3d();
      facePlane.set(faces.get(faceName));
      transformFromShapeFrame(facePlane);
      return facePlane;
   }

   /**
    * @deprecated Use setOrientation(Matrix3d)
    */
   public void setRotation(Matrix3d rotation)
   {
      super.setOrientation(rotation);
      facesAreOutOfDate = true;
   }

   /**
    * @deprecated Use setPosition(Point3d)
    */
   public void setTranslation(Point3d translation)
   {
      setPosition(translation);
      facesAreOutOfDate = true;
   }
}
