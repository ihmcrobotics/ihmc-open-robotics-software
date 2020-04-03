package us.ihmc.avatar.reachabilityMap;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class Voxel3DGrid implements ReferenceFrameHolder
{
   private final ReferenceFrame referenceFrame;
   private final BoundingBox3D boundingBox;
   private final SphereVoxelShape sphereVoxelShape;
   private final double gridSize;
   private final double voxelSize;
   private final int numberOfVoxelsPerDimension;
   private final int totalNumberOfVoxels;

   private final boolean[][][][] isRayReachable;
   private final boolean[][][][][] isPoseReachable;

   public Voxel3DGrid(ReferenceFrame referenceFrame, SphereVoxelShape sphereVoxelShape, int gridSizeInNumberOfVoxels, double voxelSize)
   {
      this.sphereVoxelShape = sphereVoxelShape;
      this.referenceFrame = referenceFrame;
      this.voxelSize = voxelSize;
      numberOfVoxelsPerDimension = gridSizeInNumberOfVoxels;
      totalNumberOfVoxels = numberOfVoxelsPerDimension * numberOfVoxelsPerDimension * numberOfVoxelsPerDimension;
      gridSize = voxelSize * gridSizeInNumberOfVoxels;
      boundingBox = new BoundingBox3D(-gridSize / 2.0, -gridSize / 2.0, -gridSize / 2.0, gridSize / 2.0, gridSize / 2.0, gridSize / 2.0);

      int numberOfRays = sphereVoxelShape.getNumberOfRays();
      int numberOfRotationsAroundRay = sphereVoxelShape.getNumberOfRotationsAroundRay();

      isRayReachable = new boolean[numberOfVoxelsPerDimension][numberOfVoxelsPerDimension][numberOfVoxelsPerDimension][numberOfRays];
      isPoseReachable = new boolean[numberOfVoxelsPerDimension][numberOfVoxelsPerDimension][numberOfVoxelsPerDimension][numberOfRays][numberOfRotationsAroundRay];
   }

   public void getVoxel(FramePoint3D voxelLocationToPack, int xIndex, int yIndex, int zIndex)
   {
      voxelLocationToPack.setToZero(referenceFrame);
      voxelLocationToPack.setX(getCoordinateFromIndex(xIndex));
      voxelLocationToPack.setY(getCoordinateFromIndex(yIndex));
      voxelLocationToPack.setZ(getCoordinateFromIndex(zIndex));
   }

   public void getClosestVoxel(FramePoint3D voxelLocationToPack, FramePoint3D inputPoint)
   {
      checkReferenceFrameMatch(inputPoint);
      if (!boundingBox.isInsideInclusive(inputPoint))
         throw new RuntimeException("The given point is outside the grid");
      voxelLocationToPack.setToZero(getReferenceFrame());
      voxelLocationToPack.setX(getCoordinateFromIndexUnsafe(getIndexFromCoordinateUnsafe(inputPoint.getX())));
      voxelLocationToPack.setY(getCoordinateFromIndexUnsafe(getIndexFromCoordinateUnsafe(inputPoint.getY())));
      voxelLocationToPack.setZ(getCoordinateFromIndexUnsafe(getIndexFromCoordinateUnsafe(inputPoint.getZ())));
   }

   private double getCoordinateFromIndex(int index)
   {
      if (index >= numberOfVoxelsPerDimension)
         throw new ArrayIndexOutOfBoundsException(index);
      return getCoordinateFromIndexUnsafe(index);
   }

   private int getIndexFromCoordinate(double coordinate)
   {
      int index = (int) (coordinate / voxelSize + numberOfVoxelsPerDimension / 2 - 1);
      if (index >= numberOfVoxelsPerDimension)
         throw new ArrayIndexOutOfBoundsException(index);
      return index;
   }

   private double getCoordinateFromIndexUnsafe(int index)
   {
      double coordinate = -gridSize / 2.0 + (index + 0.5) * voxelSize;
      return coordinate;
   }

   private int getIndexFromCoordinateUnsafe(double coordinate)
   {
      int index = (int) (coordinate / voxelSize + numberOfVoxelsPerDimension / 2 - 1);
      return index;
   }

   public void registerReachablePose(int xIndex, int yIndex, int zIndex, int rayIndex, int rotationAroundRayIndex)
   {
      boolean poseHasAlreadyBeenRegistered = isPoseReachable[xIndex][yIndex][zIndex][rayIndex][rotationAroundRayIndex];
      if (!poseHasAlreadyBeenRegistered)
      {
         isPoseReachable[xIndex][yIndex][zIndex][rayIndex][rotationAroundRayIndex] = true;
         registerReachableRay(xIndex, yIndex, zIndex, rayIndex);
      }
   }

   public void registerReachableRay(int xIndex, int yIndex, int zIndex, int rayIndex)
   {
      isRayReachable[xIndex][yIndex][zIndex][rayIndex] = true;
   }

   /**
    * Return the D reachability value in percent for this voxel based on the number of the rays that
    * have been reached.
    * 
    * @param xIndex voxel x index
    * @param yIndex voxel y index
    * @param zIndex voxel z index
    * @return The D reachability
    */
   public double getD(int xIndex, int yIndex, int zIndex)
   {
      double d = 0;
      int numberOfRays = sphereVoxelShape.getNumberOfRays();
      for (int i = 0; i < numberOfRays; i++)
      {
         if (isRayReachable[xIndex][yIndex][zIndex][i])
            d += 1.0;
      }

      d /= (double) numberOfRays;

      return d;
   }

   /**
    * Return the D0 reachability value in percent for this voxel based on the number of the
    * orientations (number of rays times number of rotations around rays) that have been reached.
    * 
    * @param xIndex voxel x index
    * @param yIndex voxel y index
    * @param zIndex voxel z index
    * @return The D0 reachability
    */
   public double getD0(int xIndex, int yIndex, int zIndex)
   {
      double d0 = 0;
      int numberOfRays = sphereVoxelShape.getNumberOfRays();
      int numberOfRotationsAroundRay = sphereVoxelShape.getNumberOfRotationsAroundRay();

      for (int i = 0; i < numberOfRays; i++)
      {
         for (int j = 0; j < numberOfRotationsAroundRay; j++)
         {
            if (isPoseReachable[xIndex][yIndex][zIndex][i][j])
               d0 += 1.0;
         }
      }

      d0 /= (double) numberOfRays;
      d0 /= (double) numberOfRotationsAroundRay;

      return d0;
   }

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   // FIXME Still in development
   private void fitCone(int xIndex, int yIndex, int zIndex)
   {
      boolean[] isRayReachable = this.isRayReachable[xIndex][yIndex][zIndex];

      List<Point3D> reachablePointsOnly = new ArrayList<>();
      for (int i = 0; i < sphereVoxelShape.getNumberOfRays(); i++)
      {
         if (isRayReachable[i])
            reachablePointsOnly.add(sphereVoxelShape.getPointsOnSphere()[i]);
      }

      pca.setPointCloud(reachablePointsOnly);
      pca.compute();
      RotationMatrix coneRotation = new RotationMatrix();
      pca.getPrincipalFrameRotationMatrix(coneRotation);

      Vector3D sphereOriginToAverage = new Vector3D();
      Vector3D thirdAxis = new Vector3D();
      coneRotation.getColumn(2, thirdAxis);

      FramePoint3D voxelLocation = new FramePoint3D();
      getVoxel(voxelLocation, xIndex, yIndex, zIndex);
      Vector3D mean = new Vector3D();
      sphereOriginToAverage.sub(mean, voxelLocation);

      if (sphereOriginToAverage.dot(thirdAxis) < 0.0)
      {
         // Rotate the frame of PI around the principal axis, such that the third axis is pointing towards the point cloud.
         RotationMatrix invertThirdAxis = new RotationMatrix();
         invertThirdAxis.setToRollOrientation(Math.PI);
         coneRotation.multiply(invertThirdAxis);
      }

      // Build the cone
      double smallestDotProduct = Double.POSITIVE_INFINITY;
      coneRotation.getColumn(2, thirdAxis);
      Vector3D testedRay = new Vector3D();
      Vector3D mostOpenedRay = new Vector3D();

      // Find the point that is the farthest from the 
      for (Point3D point : reachablePointsOnly)
      {
         testedRay.sub(point, voxelLocation);
         double absDotProduct = Math.abs(testedRay.dot(thirdAxis));
         if (absDotProduct < smallestDotProduct)
         {
            smallestDotProduct = absDotProduct;
            mostOpenedRay.set(testedRay);
         }
      }

      Vector3D standardDeviation = new Vector3D();
      pca.getStandardDeviation(standardDeviation);
      standardDeviation.scale(1.3); // Because the points are uniformly distributed

      double coneBaseRadius = Math.sqrt(standardDeviation.getX() * standardDeviation.getX() + standardDeviation.getY() * standardDeviation.getY());//radiusVector.length();
      double coneHeight = mostOpenedRay.dot(thirdAxis);

      RigidBodyTransform coneTransform = new RigidBodyTransform();

      coneTransform.getRotation().set(coneRotation);
      coneTransform.getTranslation().set(voxelLocation);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public SphereVoxelShape getSphereVoxelShape()
   {
      return sphereVoxelShape;
   }

   public double getGridSize()
   {
      return gridSize;
   }

   public double getVoxelSize()
   {
      return voxelSize;
   }

   public int getNumberOfVoxelsPerDimension()
   {
      return numberOfVoxelsPerDimension;
   }

   public int getTotalNumberOfVoxels()
   {
      return totalNumberOfVoxels;
   }

   public FramePoint3D getMinPoint()
   {
      return new FramePoint3D(referenceFrame, boundingBox.getMinPoint());
   }

   public FramePoint3D getMaxPoint()
   {
      return new FramePoint3D(referenceFrame, boundingBox.getMaxPoint());
   }
}
