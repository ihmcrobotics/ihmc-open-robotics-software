package us.ihmc.avatar.reachabilityMap;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
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

   private final Voxel3DData[] voxels;

   public Voxel3DGrid(ReferenceFrame referenceFrame, SphereVoxelShape sphereVoxelShape, int gridSizeInNumberOfVoxels, double voxelSize)
   {
      this.sphereVoxelShape = sphereVoxelShape;
      this.referenceFrame = referenceFrame;
      this.voxelSize = voxelSize;
      numberOfVoxelsPerDimension = gridSizeInNumberOfVoxels;
      totalNumberOfVoxels = numberOfVoxelsPerDimension * numberOfVoxelsPerDimension * numberOfVoxelsPerDimension;
      gridSize = voxelSize * gridSizeInNumberOfVoxels;
      boundingBox = new BoundingBox3D(-gridSize / 2.0, -gridSize / 2.0, -gridSize / 2.0, gridSize / 2.0, gridSize / 2.0, gridSize / 2.0);
      voxels = new Voxel3DData[numberOfVoxelsPerDimension * numberOfVoxelsPerDimension * numberOfVoxelsPerDimension];
   }

   public Voxel3DData getVoxel(FrameTuple3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return getVoxel((Tuple3DReadOnly) query);
   }

   public Voxel3DData getVoxel(Tuple3DReadOnly query)
   {
      return getVoxel(query.getX(), query.getY(), query.getZ());
   }

   public Voxel3DData getVoxel(double x, double y, double z)
   {
      if (!boundingBox.isInsideInclusive(x, y, z))
         throw new IllegalArgumentException("The given point is outside the grid");
      return getVoxel(toIndex(x), toIndex(y), toIndex(z));
   }

   public Voxel3DData getVoxel(int xIndex, int yIndex, int zIndex)
   {
      return voxels[(xIndex * numberOfVoxelsPerDimension + yIndex) * numberOfVoxelsPerDimension + zIndex];
   }

   public Voxel3DData getOrCreateVoxel(int xIndex, int yIndex, int zIndex)
   {
      int index = (xIndex * numberOfVoxelsPerDimension + yIndex) * numberOfVoxelsPerDimension + zIndex;
      Voxel3DData voxel = voxels[index];
      if (voxel == null)
      {
         voxel = new Voxel3DData(xIndex, yIndex, zIndex);
         voxels[index] = voxel;
      }
      return voxel;
   }

   public void destroy(Voxel3DData voxel)
   {
      voxels[(voxel.xIndex * numberOfVoxelsPerDimension + voxel.yIndex) * numberOfVoxelsPerDimension + voxel.zIndex] = null;
   }

   private double toCoordinate(int index)
   {
      return (index + 0.5) * voxelSize - 0.5 * gridSize;
   }

   private int toIndex(double coordinate)
   {
      return (int) (coordinate / voxelSize + numberOfVoxelsPerDimension / 2 - 1);
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
      Voxel3DData voxel = getVoxel(xIndex, yIndex, zIndex);
      return voxel == null ? 0 : voxel.getD();
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
      Voxel3DData voxel = getVoxel(xIndex, yIndex, zIndex);
      return voxel == null ? 0 : voxel.getD0();
   }

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   // FIXME Still in development
   private void fitCone(int xIndex, int yIndex, int zIndex)
   {
      boolean[] isRayReachable = getVoxel(xIndex, yIndex, zIndex).isRayReachable;

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

      FramePoint3D voxelLocation = new FramePoint3D(getVoxel(xIndex, yIndex, zIndex).getPosition());
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

   public class Voxel3DData
   {
      private final int xIndex, yIndex, zIndex;
      private final FramePoint3DReadOnly position;

      private boolean[] isRayReachable;
      private boolean[][] isPoseReachable;

      public Voxel3DData(int xIndex, int yIndex, int zIndex)
      {
         if (xIndex >= numberOfVoxelsPerDimension)
            throw new ArrayIndexOutOfBoundsException(xIndex);
         if (yIndex >= numberOfVoxelsPerDimension)
            throw new ArrayIndexOutOfBoundsException(yIndex);
         if (zIndex >= numberOfVoxelsPerDimension)
            throw new ArrayIndexOutOfBoundsException(zIndex);

         this.xIndex = xIndex;
         this.yIndex = yIndex;
         this.zIndex = zIndex;

         position = EuclidFrameFactories.newLinkedFramePoint3DReadOnly(Voxel3DGrid.this,
                                                                       () -> toCoordinate(xIndex),
                                                                       () -> toCoordinate(yIndex),
                                                                       () -> toCoordinate(zIndex));
      }

      public void registerReachablePose(int rayIndex, int rotationAroundRayIndex)
      {
         if (isPoseReachable == null)
            isPoseReachable = new boolean[sphereVoxelShape.getNumberOfRays()][sphereVoxelShape.getNumberOfRotationsAroundRay()];

         boolean poseHasAlreadyBeenRegistered = isPoseReachable[rayIndex][rotationAroundRayIndex];

         if (!poseHasAlreadyBeenRegistered)
         {
            isPoseReachable[rayIndex][rotationAroundRayIndex] = true;
            registerReachableRay(rayIndex);
         }
      }

      public void registerReachableRay(int rayIndex)
      {
         if (isRayReachable == null)
            isRayReachable = new boolean[sphereVoxelShape.getNumberOfRays()];
         isRayReachable[rayIndex] = true;
      }

      /**
       * Return the D reachability value in percent for this voxel based on the number of the rays that
       * have been reached.
       * 
       * @return The D reachability
       */
      public double getD()
      {
         if (isRayReachable == null)
            return 0;

         double d = 0;
         int numberOfRays = sphereVoxelShape.getNumberOfRays();

         for (int i = 0; i < numberOfRays; i++)
         {
            if (isRayReachable[i])
               d += 1.0;
         }

         d /= (double) numberOfRays;

         return d;
      }

      /**
       * Return the D0 reachability value in percent for this voxel based on the number of the
       * orientations (number of rays times number of rotations around rays) that have been reached.
       * 
       * @return The D0 reachability
       */
      public double getD0()
      {
         if (isPoseReachable == null)
            return 0;

         double d0 = 0;
         int numberOfRays = sphereVoxelShape.getNumberOfRays();
         int numberOfRotationsAroundRay = sphereVoxelShape.getNumberOfRotationsAroundRay();

         for (int i = 0; i < numberOfRays; i++)
         {
            for (int j = 0; j < numberOfRotationsAroundRay; j++)
            {
               if (isPoseReachable[i][j])
                  d0 += 1.0;
            }
         }

         d0 /= (double) numberOfRays;
         d0 /= (double) numberOfRotationsAroundRay;

         return d0;
      }

      public FramePoint3DReadOnly getPosition()
      {
         return position;
      }
   }
}
