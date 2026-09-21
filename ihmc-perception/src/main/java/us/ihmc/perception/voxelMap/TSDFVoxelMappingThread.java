package us.ihmc.perception.voxelMap;

import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.VoxelMapMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.filters.DepthImageBodyCollisionFilter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.sensors.ImageSensor;

import javax.annotation.Nullable;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;

import static us.ihmc.perception.voxelMap.TSDFVoxelMapExtractor.FREE_SPACE_VALUE;

/**
 * Publishes a temporally-accumulated, ego-motion-warped robot-centric TSDF grid at camera frame rate.
 *
 * <p>Mirrors {@code RobotCentricTSDFFromDepthTerm} from {@code depth_voxel_tsdf.py} exactly:
 * on every camera update, the previous TSDF memory is backward-warped into the current robot pose
 * (trilinear interpolation, out-of-bounds voxels reset to free space), then overwritten wherever
 * the depth cameras actually observe a measurement, and published as a {@link VoxelMapMessage}
 * on the configured ROS 2 topic.</p>
 *
 * <p>Between camera updates, the same grid snapshot is published unchanged -- this matches the
 * deployed system's architecture where the control computer receives the last published snapshot
 * without re-warping, ensuring training and deployment see the same temporal behavior.</p>
 *
 * <p>Drop-in replacement for {@link VoxelMappingThread} for TSDF-trained policies.  The published
 * message uses the same {@link VoxelMapMessage} type (float32[] voxel_map_data of length N^3),
 * so the RLEstimates subscriber requires no changes.</p>
 */
public class TSDFVoxelMappingThread extends RepeatingTaskThread
{
   private final Map<ImageSensor, Integer> sensorDepthImageKeyMap;
   private final Supplier<RigidBodyTransformReadOnly> mapOriginSupplier;

   private final TSDFVoxelMapExtractor extractor;
   @Nullable
   private final DepthImageBodyCollisionFilter bodyCollisionFilter;

   private final ROS2Node ros2Node;
   private final ROS2Publisher<VoxelMapMessage> publisher;
   private final VoxelMapMessage voxelMapMessage;

   private final int mapSize;
   private final float voxelSize;
   private final int voxelCount;

   // Temporal memory: previous merged TSDF grid (CPU float array) and the robot pose it was built at.
   // Initialized to FREE_SPACE_VALUE (matching Python's torch.full(..., _FREE_SPACE_VALUE)).
   private final float[] previousTSDFGrid;
   private final RigidBodyTransform previousOrigin = new RigidBodyTransform();

   // Scratch buffers for the warp, reused across calls
   private final float[] warpedGrid;

   public TSDFVoxelMappingThread(ROS2Node ros2Node,
                                 ROS2Topic<VoxelMapMessage> ros2Topic,
                                 int mapSize,
                                 float voxelSize,
                                 float truncationDistance,
                                 Supplier<RigidBodyTransformReadOnly> mapOriginSupplier,
                                 Map<ImageSensor, Integer> sensorDepthImageKeyMap)
   {
      this(ros2Node, ros2Topic, mapSize, voxelSize, truncationDistance, mapOriginSupplier, sensorDepthImageKeyMap, null, null);
   }

   public TSDFVoxelMappingThread(ROS2Node ros2Node,
                                 ROS2Topic<VoxelMapMessage> ros2Topic,
                                 int mapSize,
                                 float voxelSize,
                                 float truncationDistance,
                                 Supplier<RigidBodyTransformReadOnly> mapOriginSupplier,
                                 Map<ImageSensor, Integer> sensorDepthImageKeyMap,
                                 @Nullable RobotCollisionModel robotCollisionModel,
                                 @Nullable RigidBodyBasics rootBody)
   {
      super(TSDFVoxelMappingThread.class.getSimpleName());

      this.ros2Node = ros2Node;
      publisher = ros2Node.createPublisher(ros2Topic);
      voxelMapMessage = new VoxelMapMessage();

      this.mapOriginSupplier = mapOriginSupplier;
      this.sensorDepthImageKeyMap = sensorDepthImageKeyMap;
      this.mapSize = mapSize;
      this.voxelSize = voxelSize;
      this.voxelCount = mapSize * mapSize * mapSize;

      extractor = new TSDFVoxelMapExtractor(mapSize, voxelSize, truncationDistance);

      bodyCollisionFilter = (robotCollisionModel != null && rootBody != null)
            ? new DepthImageBodyCollisionFilter(robotCollisionModel, rootBody)
            : null;

      previousTSDFGrid = new float[voxelCount];
      Arrays.fill(previousTSDFGrid, FREE_SPACE_VALUE);
      warpedGrid = new float[voxelCount];
   }

   @Override
   protected void runTask()
   {
      RawImage[] depthImages = new RawImage[sensorDepthImageKeyMap.size()];
      int arrayIndex = 0;
      for (ImageSensor imageSensor : sensorDepthImageKeyMap.keySet())
      {
         RawImage depthImage = imageSensor.getImage(sensorDepthImageKeyMap.get(imageSensor));
         if (depthImage != null)
            depthImages[arrayIndex++] = depthImage;
      }

      if (arrayIndex == 0)
         return;

      if (bodyCollisionFilter != null)
      {
         for (int i = 0; i < arrayIndex; i++)
         {
            RawImage raw = depthImages[i];
            GpuMat rawGpu = raw.getGpuImageMat();
            GpuMat filteredGpu = new GpuMat(rawGpu.size(), rawGpu.type());
            ReferenceFrame cameraFrame = new FixedReferenceFrame("TSDFCameraFrame", ReferenceFrame.getWorldFrame(), raw.getTransformToWorld());
            bodyCollisionFilter.process(rawGpu, filteredGpu, raw.getIntrinsicsCopy(), cameraFrame);
            depthImages[i] = raw.replaceImage(filteredGpu);
            raw.release();
         }
      }

      RigidBodyTransformReadOnly currentOrigin = mapOriginSupplier.get();

      // 1. Compute instant TSDF from current depth images on GPU
      TSDFVoxelMap instantMap = extractor.getTSDFVoxelMap(currentOrigin, depthImages);
      float[] instantTSDF = instantMap.getCpuData();
      byte[] validMask = instantMap.getValidCpuData();
      instantMap.close();

      // 2. Warp previous memory from previousOrigin to currentOrigin (CPU trilinear interpolation).
      //    On the very first call, previousOrigin == currentOrigin, so the warp is an identity
      //    pass-through of the all-FREE_SPACE_VALUE initial grid.
      warpTSDFGrid(previousTSDFGrid, previousOrigin, currentOrigin, warpedGrid, mapSize, voxelSize);

      // 3. Merge: where a camera observed a voxel, use the instant TSDF; elsewhere keep warped memory.
      //    Matches Python: fresh_rows = torch.where(instant_valid, instant_tsdf, warped_memory)
      for (int i = 0; i < voxelCount; i++)
         previousTSDFGrid[i] = (validMask[i] != 0) ? instantTSDF[i] : warpedGrid[i];

      // 4. Publish merged grid
      packAndPublish(previousTSDFGrid, currentOrigin);

      // 5. Advance temporal state: store current pose for the next warp
      previousOrigin.set(currentOrigin);

      for (RawImage image : depthImages)
         if (image != null)
            image.release();
   }

   /**
    * Backward-warps {@code prevGrid} from {@code prevOrigin} to {@code currOrigin} via trilinear
    * interpolation, matching {@code depth_voxel_tsdf.warp_tsdf_grid} exactly.
    * Package-private and static so unit tests can call it without a full thread instance.
    */
   static void warpTSDFGrid(float[] prevGrid,
                             RigidBodyTransformReadOnly prevOrigin,
                             RigidBodyTransformReadOnly currOrigin,
                             float[] outputGrid,
                             int mapSize,
                             float voxelSize)
   {
      // currMapToPrevMap = prevOrigin^{-1} * currOrigin
      // Takes a point expressed in the current map frame and returns it in the previous map frame.
      RigidBodyTransform currMapToPrevMap = new RigidBodyTransform(prevOrigin);
      currMapToPrevMap.invert();
      currMapToPrevMap.multiply(currOrigin);

      RotationMatrixReadOnly rot = currMapToPrevMap.getRotation();
      Vector3DReadOnly trans = currMapToPrevMap.getTranslation();

      int n = mapSize;
      float vs = voxelSize;
      float halfNm1 = (n - 1) * 0.5f;

      for (int ix = 0; ix < n; ix++)
      {
         for (int iy = 0; iy < n; iy++)
         {
            for (int iz = 0; iz < n; iz++)
            {
               // Voxel centre in current map frame
               float vx = (ix - halfNm1) * vs;
               float vy = (iy - halfNm1) * vs;
               float vz = (iz - halfNm1) * vs;

               // Transform to previous map frame: R * p + t
               float px = (float) (rot.getM00() * vx + rot.getM01() * vy + rot.getM02() * vz + trans.getX());
               float py = (float) (rot.getM10() * vx + rot.getM11() * vy + rot.getM12() * vz + trans.getY());
               float pz = (float) (rot.getM20() * vx + rot.getM21() * vy + rot.getM22() * vz + trans.getZ());

               // Convert to fractional grid coordinates [0, n-1].
               // align_corners=True: voxel centre at ±halfNm1*vs maps to grid coord 0 or n-1.
               float gx = px / vs + halfNm1;
               float gy = py / vs + halfNm1;
               float gz = pz / vs + halfNm1;

               int outIdx = (ix * n + iy) * n + iz;

               // Out-of-bounds: free space (matching Python's oob_mask replacement)
               if (gx < 0.0f || gx > n - 1 || gy < 0.0f || gy > n - 1 || gz < 0.0f || gz > n - 1)
               {
                  outputGrid[outIdx] = FREE_SPACE_VALUE;
                  continue;
               }

               // Trilinear interpolation (8-corner stencil, matching grid_sample bilinear in 3D)
               int x0 = (int) gx;
               int y0 = (int) gy;
               int z0 = (int) gz;
               int x1 = Math.min(x0 + 1, n - 1);
               int y1 = Math.min(y0 + 1, n - 1);
               int z1 = Math.min(z0 + 1, n - 1);

               float dx = gx - x0;
               float dy = gy - y0;
               float dz = gz - z0;
               float nx = 1.0f - dx;
               float ny = 1.0f - dy;
               float nz = 1.0f - dz;

               float v000 = prevGrid[(x0 * n + y0) * n + z0];
               float v100 = prevGrid[(x1 * n + y0) * n + z0];
               float v010 = prevGrid[(x0 * n + y1) * n + z0];
               float v110 = prevGrid[(x1 * n + y1) * n + z0];
               float v001 = prevGrid[(x0 * n + y0) * n + z1];
               float v101 = prevGrid[(x1 * n + y0) * n + z1];
               float v011 = prevGrid[(x0 * n + y1) * n + z1];
               float v111 = prevGrid[(x1 * n + y1) * n + z1];

               outputGrid[outIdx] = nx * ny * nz * v000
                                  + dx * ny * nz * v100
                                  + nx * dy * nz * v010
                                  + dx * dy * nz * v110
                                  + nx * ny * dz * v001
                                  + dx * ny * dz * v101
                                  + nx * dy * dz * v011
                                  + dx * dy * dz * v111;
            }
         }
      }
   }

   private void packAndPublish(float[] mergedGrid, RigidBodyTransformReadOnly origin)
   {
      voxelMapMessage.getVoxelMapData().clear();
      voxelMapMessage.getVoxelMapData().ensureMinCapacity(voxelCount);
      voxelMapMessage.getVoxelMapData().getBuffer().position(0);
      float[] buf = voxelMapMessage.getVoxelMapData().getBuffer().array();
      System.arraycopy(mergedGrid, 0, buf, 0, voxelCount);
      voxelMapMessage.getVoxelMapData().getBuffer().position(voxelCount);
      voxelMapMessage.setSizeX(mapSize);
      voxelMapMessage.setSizeY(mapSize);
      voxelMapMessage.setSizeZ(mapSize);
      voxelMapMessage.setVoxelSize(voxelSize);
      voxelMapMessage.getOrigin().set(origin);
      publisher.publish(voxelMapMessage);
   }

   @Override
   public void kill()
   {
      super.kill();
      extractor.close();
      if (bodyCollisionFilter != null)
         bodyCollisionFilter.close();
      if (publisher != null)
         ros2Node.destroyPublisher(publisher);
   }
}
