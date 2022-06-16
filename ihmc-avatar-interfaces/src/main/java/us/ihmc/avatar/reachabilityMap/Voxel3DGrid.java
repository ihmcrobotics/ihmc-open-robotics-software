package us.ihmc.avatar.reachabilityMap;

import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class Voxel3DGrid implements ReferenceFrameHolder
{
   public static final int MAX_GRID_SIZE_VOXELS = (int) Math.pow(Integer.MAX_VALUE, 1.0 / 3.0);

   private final BoundingBox3D boundingBox;
   private final SphereVoxelShape sphereVoxelShape;
   private final double voxelSize;
   private final double gridSizeMeters;
   private final int gridSizeVoxels;
   private final int numberOfVoxels;
   private final Voxel3DData[] voxels;

   private final PoseReferenceFrame referenceFrame;

   public static Voxel3DGrid newVoxel3DGrid(int gridSizeInNumberOfVoxels, double voxelSize, int numberOfRays, int numberOfRotationsAroundRay)
   {
      SphereVoxelShape sphereVoxelShape = new SphereVoxelShape(voxelSize, numberOfRays, numberOfRotationsAroundRay, SphereVoxelType.graspOrigin);
      return new Voxel3DGrid(sphereVoxelShape, gridSizeInNumberOfVoxels, voxelSize);
   }

   public Voxel3DGrid(SphereVoxelShape sphereVoxelShape, int gridSizeInNumberOfVoxels, double voxelSize)
   {
      if (gridSizeInNumberOfVoxels > MAX_GRID_SIZE_VOXELS)
         throw new IllegalArgumentException("Grid size is too big: " + gridSizeInNumberOfVoxels + " [max=" + MAX_GRID_SIZE_VOXELS + "]");

      this.sphereVoxelShape = sphereVoxelShape;
      this.voxelSize = voxelSize;
      gridSizeVoxels = gridSizeInNumberOfVoxels;

      numberOfVoxels = gridSizeVoxels * gridSizeVoxels * gridSizeVoxels;

      gridSizeMeters = voxelSize * gridSizeInNumberOfVoxels;
      double halfSize = gridSizeMeters / 2.0;
      boundingBox = new BoundingBox3D(-halfSize, -halfSize, -halfSize, halfSize, halfSize, halfSize);
      voxels = new Voxel3DData[numberOfVoxels];
      referenceFrame = new PoseReferenceFrame("voxel3DGridFrame", ReferenceFrame.getWorldFrame());
      sphereVoxelShape.setReferenceFrame(referenceFrame);
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
      return voxels[Voxel3DKey.toArrayIndex(xIndex, yIndex, zIndex, gridSizeVoxels)];
   }

   public Voxel3DData getVoxel(int index)
   {
      return voxels[index];
   }

   public Voxel3DData getOrCreateVoxel(int index)
   {
      Voxel3DData voxel = voxels[index];

      if (voxel == null)
      {
         voxel = new Voxel3DData(new Voxel3DKey(index, gridSizeVoxels));
         voxels[index] = voxel;
      }

      return voxel;
   }

   public Voxel3DData getOrCreateVoxel(int xIndex, int yIndex, int zIndex)
   {
      int index = Voxel3DKey.toArrayIndex(xIndex, yIndex, zIndex, gridSizeVoxels);
      Voxel3DData voxel = voxels[index];
      if (voxel == null)
      {
         voxel = new Voxel3DData(new Voxel3DKey(xIndex, yIndex, zIndex, gridSizeVoxels));
         voxels[index] = voxel;
      }
      return voxel;
   }

   public void destroy(Voxel3DData voxel)
   {
      voxels[voxel.getKey().getIndex()] = null;
   }

   public FramePoint3DReadOnly getVoxelPosition(int index)
   {
      return new Voxel3DData(new Voxel3DKey(index, gridSizeVoxels)).getPosition();
   }

   private double toCoordinate(int index)
   {
      return (index + 0.5) * voxelSize - 0.5 * gridSizeMeters;
   }

   private int toIndex(double coordinate)
   {
      return (int) (coordinate / voxelSize + gridSizeVoxels / 2 - 1);
   }

   public void setGridPose(RigidBodyTransformReadOnly pose)
   {
      referenceFrame.setPoseAndUpdate(pose);
   }

   public void setGridPose(Pose3DReadOnly pose)
   {
      referenceFrame.setPoseAndUpdate(pose);
   }

   @Override
   public PoseReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public SphereVoxelShape getSphereVoxelShape()
   {
      return sphereVoxelShape;
   }

   public double getVoxelSize()
   {
      return voxelSize;
   }

   public double getGridSizeMeters()
   {
      return gridSizeMeters;
   }

   public int getGridSizeVoxels()
   {
      return gridSizeVoxels;
   }

   public int getNumberOfVoxels()
   {
      return numberOfVoxels;
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
      private final Voxel3DKey key;
      private final FramePoint3DReadOnly position;

      private VoxelExtraData positionExtraData;
      private VoxelExtraData[] rayExtraData;
      private VoxelExtraData[] poseExtraData;

      public Voxel3DData(Voxel3DKey key)
      {
         this.key = key;
         position = new FramePoint3D(getReferenceFrame(), toCoordinate(key.x), toCoordinate(key.y), toCoordinate(key.z));
      }

      public void registerReachablePosition(Point3DReadOnly desiredPosition, float[] jointPositions, float[] jointTorques)
      {
         if ((jointPositions == null || jointPositions.length == 0) && (jointTorques == null || jointTorques.length == 0))
            return;

         positionExtraData = new VoxelExtraData();
         positionExtraData.setDesiredPosition(desiredPosition);
         positionExtraData.jointPositions = jointPositions;
         positionExtraData.jointTorques = jointTorques;
      }

      public void registerReachablePosition(Point3DReadOnly desiredPosition, OneDoFJointReadOnly[] joints)
      {
         positionExtraData = new VoxelExtraData();
         positionExtraData.setDesiredPosition(desiredPosition);
         positionExtraData.setJointPositions(joints);
         positionExtraData.setJointTorques(joints);
      }

      public void registerReachableRay(int rayIndex, Pose3DReadOnly desiredPose, float[] jointPositions, float[] jointTorques)
      {
         if (desiredPose == null && (jointPositions == null || jointPositions.length == 0) && (jointTorques == null || jointTorques.length == 0))
            return;

         if (rayExtraData == null)
            rayExtraData = new VoxelExtraData[getNumberOfRays()];

         VoxelExtraData jointData = new VoxelExtraData();
         jointData.setDesiredPose(desiredPose);
         jointData.jointPositions = jointPositions;
         jointData.jointTorques = jointTorques;
         rayExtraData[rayIndex] = jointData;
      }

      public void registerReachableRay(int rayIndex, Pose3DReadOnly desiredPose, OneDoFJointReadOnly[] joints)
      {
         if (rayExtraData == null)
            rayExtraData = new VoxelExtraData[getNumberOfRays()];

         VoxelExtraData extraData = new VoxelExtraData();
         extraData.setDesiredPose(desiredPose);
         extraData.setJointPositions(joints);
         extraData.setJointTorques(joints);
         rayExtraData[rayIndex] = extraData;
      }

      public void registerReachablePose(int rayIndex, int rotationAroundRayIndex, Pose3DReadOnly desiredPose, float[] jointPositions, float[] jointTorques)
      {
         if (desiredPose == null && (jointPositions == null || jointPositions.length == 0) && (jointTorques == null || jointTorques.length == 0))
            return;

         if (poseExtraData == null)
            poseExtraData = new VoxelExtraData[getNumberOfRays() * getNumberOfRotationsAroundRay()];

         VoxelExtraData extraData = new VoxelExtraData();
         extraData.setDesiredPose(desiredPose);
         extraData.jointPositions = jointPositions;
         extraData.jointTorques = jointTorques;
         poseExtraData[rayIndex * getNumberOfRotationsAroundRay() + rotationAroundRayIndex] = extraData;
      }

      public void registerReachablePose(int rayIndex, int rotationAroundRayIndex, Pose3DReadOnly desiredPose, OneDoFJointReadOnly[] joints)
      {
         if (poseExtraData == null)
            poseExtraData = new VoxelExtraData[getNumberOfRays() * getNumberOfRotationsAroundRay()];

         VoxelExtraData extraData = new VoxelExtraData();
         extraData.setDesiredPose(desiredPose);
         extraData.setJointPositions(joints);
         extraData.setJointTorques(joints);
         poseExtraData[rayIndex * getNumberOfRotationsAroundRay() + rotationAroundRayIndex] = extraData;
      }

      /**
       * Return the R reachability value in percent for this voxel based on the number of the rays that
       * have been reached.
       * 
       * @return The R reachability
       */
      public double getR()
      {
         return (double) getNumberOfReachableRays() / (double) getNumberOfRays();
      }

      public int getNumberOfReachableRays()
      {
         if (rayExtraData == null)
            return 0;

         int n = 0;
         int numberOfRays = getNumberOfRays();

         for (int rayIndex = 0; rayIndex < numberOfRays; rayIndex++)
         {
            if (isRayReachable(rayIndex))
               n++;
         }

         return n;
      }

      /**
       * Return the R2 reachability value in percent for this voxel based on the number of the
       * orientations (number of rays times number of rotations around rays) that have been reached.
       * 
       * @return The R2 reachability
       */
      public double getR2()
      {
         return (double) getNumberOfReachablePoses() / (double) getNumberOfRays() / (double) getNumberOfRotationsAroundRay();
      }

      public int getNumberOfReachablePoses()
      {
         if (poseExtraData == null)
            return 0;

         int n = 0;

         for (int rayIndex = 0; rayIndex < getNumberOfRays(); rayIndex++)
         {
            n += getNumberOfReachableRotationsAroundRay(rayIndex);
         }

         return n;
      }

      public int getNumberOfReachableRotationsAroundRay(int rayIndex)
      {
         if (poseExtraData == null)
            return 0;

         int n = 0;

         for (int rotationIndex = 0; rotationIndex < getNumberOfRotationsAroundRay(); rotationIndex++)
         {
            if (isPoseReachable(rayIndex, rotationIndex))
               n++;
         }

         return n;
      }

      public double computeD06()
      {
         int nCommOrientations = 0;

         for (int rayIndex = 0; rayIndex < getNumberOfRays(); rayIndex++)
         {
            for (int rotationIndex = 0; rotationIndex < getNumberOfRotationsAroundRay(); rotationIndex++)
            {
               nCommOrientations += compute6NeighborCommonOrientation(rayIndex, rotationIndex);
            }
         }

         return (double) nCommOrientations / (6.0 * getNumberOfReachablePoses());
      }

      public double computeD018()
      {
         int nCommOrientations = 0;

         for (int rayIndex = 0; rayIndex < getNumberOfRays(); rayIndex++)
         {
            for (int rotationIndex = 0; rotationIndex < getNumberOfRotationsAroundRay(); rotationIndex++)
            {
               nCommOrientations += compute18NeighborCommonOrientation(rayIndex, rotationIndex);
            }
         }

         return (double) nCommOrientations / (18.0 * getNumberOfReachablePoses());
      }

      public double computeD026()
      {
         int nCommOrientations = 0;

         for (int rayIndex = 0; rayIndex < getNumberOfRays(); rayIndex++)
         {
            for (int rotationIndex = 0; rotationIndex < getNumberOfRotationsAroundRay(); rotationIndex++)
            {
               nCommOrientations += compute26NeighborCommonOrientation(rayIndex, rotationIndex);
            }
         }

         return (double) nCommOrientations / (26.0 * getNumberOfReachablePoses());
      }

      public int compute6NeighborCommonOrientation(int rayIndex, int rotationIndex)
      {
         int n = 0;
         n += isNeightPoseReachable(+1, 0, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, 0, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, +1, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, -1, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, 0, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, 0, -1, rayIndex, rotationIndex) ? 1 : 0;
         return n;
      }

      public int compute18NeighborCommonOrientation(int rayIndex, int rotationIndex)
      {
         int n = compute6NeighborCommonOrientation(rayIndex, rotationIndex);
         n += isNeightPoseReachable(+1, +1, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(+1, -1, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, +1, 0, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, -1, 0, rayIndex, rotationIndex) ? 1 : 0;

         n += isNeightPoseReachable(0, +1, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, +1, -1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, -1, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(0, -1, -1, rayIndex, rotationIndex) ? 1 : 0;

         n += isNeightPoseReachable(+1, 0, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, 0, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(+1, 0, -1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, 0, -1, rayIndex, rotationIndex) ? 1 : 0;

         return n;
      }

      public int compute26NeighborCommonOrientation(int rayIndex, int rotationIndex)
      {
         int n = compute18NeighborCommonOrientation(rayIndex, rotationIndex);
         n += isNeightPoseReachable(+1, +1, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(+1, +1, -1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(+1, -1, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(+1, -1, -1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, +1, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, +1, -1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, -1, +1, rayIndex, rotationIndex) ? 1 : 0;
         n += isNeightPoseReachable(-1, -1, -1, rayIndex, rotationIndex) ? 1 : 0;

         return n;
      }

      public boolean isNeightPoseReachable(int xShift, int yShift, int zShift, int rayIndex, int rotationIndex)
      {
         Voxel3DData neighbor = getNeighbor(1, 0, 0);
         return neighbor != null && neighbor.isPoseReachable(rayIndex, rotationIndex);
      }

      public Voxel3DData getNeighbor(int xShift, int yShift, int zShift)
      {
         return getVoxel(key.getX() + xShift, key.getY() + yShift, key.getZ() + zShift);
      }

      public double getSize()
      {
         return voxelSize;
      }

      public Voxel3DKey getKey()
      {
         return key;
      }

      public FramePoint3DReadOnly getPosition()
      {
         return position;
      }

      public SphereVoxelShape getSphereVoxelShape()
      {
         return sphereVoxelShape;
      }

      public int getNumberOfRays()
      {
         return sphereVoxelShape.getNumberOfRays();
      }

      public int getNumberOfRotationsAroundRay()
      {
         return sphereVoxelShape.getNumberOfRotationsAroundRay();
      }

      public boolean atLeastOneReachableRay()
      {
         if (rayExtraData == null)
            return false;
         for (VoxelExtraData jointData : rayExtraData)
         {
            if (jointData != null)
               return true;
         }
         return false;
      }

      public boolean isRayReachable(int rayIndex)
      {
         return rayExtraData == null ? false : rayExtraData[rayIndex] != null;
      }

      public boolean atLeastOneReachablePose()
      {
         if (poseExtraData == null)
            return false;
         for (VoxelExtraData jointData : poseExtraData)
         {
            if (jointData != null)
               return true;
         }
         return false;
      }

      public boolean isPoseReachable(int rayIndex, int rotationIndex)
      {
         return getPoseExtraData(rayIndex, rotationIndex) != null;
      }

      public VoxelExtraData getPositionExtraData()
      {
         return positionExtraData;
      }

      public VoxelExtraData getRayExtraData(int rayIndex)
      {
         if (rayExtraData == null)
            return null;
         return rayExtraData[rayIndex];
      }

      public VoxelExtraData getPoseExtraData(int rayIndex, int rotationIndex)
      {
         if (poseExtraData == null)
            return null;
         return poseExtraData[rayIndex * getNumberOfRotationsAroundRay() + rotationIndex];
      }
   }

   public static class VoxelExtraData
   {
      private Point3D desiredPosition;
      private Quaternion desiredOrientation;
      private float[] jointPositions;
      private float[] jointTorques;

      public VoxelExtraData()
      {
      }

      public void setDesiredPosition(Point3DReadOnly desiredPosition)
      {
         this.desiredPosition = new Point3D(desiredPosition);
         this.desiredOrientation = null;
      }

      public void setDesiredPose(Pose3DReadOnly desiredPose)
      {
         this.desiredPosition = new Point3D(desiredPose.getPosition());
         this.desiredOrientation = new Quaternion(desiredPose.getOrientation());
      }

      public void setJointPositions(OneDoFJointReadOnly[] joints)
      {
         if (jointPositions == null)
            jointPositions = new float[joints.length];
         for (int i = 0; i < joints.length; i++)
         {
            jointPositions[i] = (float) joints[i].getQ();
         }
      }

      public void setJointTorques(OneDoFJointReadOnly[] joints)
      {
         if (jointTorques == null)
            jointTorques = new float[joints.length];
         for (int i = 0; i < joints.length; i++)
         {
            jointTorques[i] = (float) joints[i].getTau();
         }
      }

      public Point3D getDesiredPosition()
      {
         return desiredPosition;
      }

      public Quaternion getDesiredOrientation()
      {
         return desiredOrientation;
      }

      public float[] getJointPositions()
      {
         return jointPositions;
      }

      public float[] getJointTorques()
      {
         return jointTorques;
      }
   }

   public static class Voxel3DKey
   {
      private int x, y, z;
      private int index;

      public Voxel3DKey(int x, int y, int z, int gridSizeVoxels)
      {
         if (x >= gridSizeVoxels)
            throw new ArrayIndexOutOfBoundsException(x);
         if (y >= gridSizeVoxels)
            throw new ArrayIndexOutOfBoundsException(y);
         if (z >= gridSizeVoxels)
            throw new ArrayIndexOutOfBoundsException(z);

         this.x = x;
         this.y = y;
         this.z = z;
         index = toArrayIndex(x, y, z, gridSizeVoxels);
      }

      public Voxel3DKey(int index, int gridSizeVoxels)
      {
         this.index = index;
         x = toXindex(index, gridSizeVoxels);
         y = toYindex(index, gridSizeVoxels);
         z = toZindex(index, gridSizeVoxels);

         if (x >= gridSizeVoxels)
            throw new ArrayIndexOutOfBoundsException(x);
         if (y >= gridSizeVoxels)
            throw new ArrayIndexOutOfBoundsException(y);
         if (z >= gridSizeVoxels)
            throw new ArrayIndexOutOfBoundsException(z);
      }

      public static int toArrayIndex(int x, int y, int z, int gridSizeVoxels)
      {
         return (x * gridSizeVoxels + y) * gridSizeVoxels + z;
      }

      public static int toXindex(int arrayIndex, int gridSizeVoxels)
      {
         return arrayIndex / gridSizeVoxels / gridSizeVoxels;
      }

      public static int toYindex(int arrayIndex, int gridSizeVoxels)
      {
         return (arrayIndex / gridSizeVoxels) % gridSizeVoxels;
      }

      public static int toZindex(int arrayIndex, int gridSizeVoxels)
      {
         return arrayIndex % gridSizeVoxels;
      }

      @Override
      public int hashCode()
      {
         return index;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof Voxel3DKey)
         {
            Voxel3DKey other = (Voxel3DKey) object;
            return x == other.x && y == other.y && z == other.z;
         }
         else
         {
            return false;
         }
      }

      public int getX()
      {
         return x;
      }

      public int getY()
      {
         return y;
      }

      public int getZ()
      {
         return z;
      }

      public int getIndex()
      {
         return index;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getStringOf("(", ")", ", ", x, y, z);
      }
   }
}
