package us.ihmc.perception;

import perception_msgs.msg.dds.Float32MultiArrayHack;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.VoxelOccupancyPacking;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAGPUVoxelGrid;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

/**
 * Extracts point clouds from both ZED X Mini depth cameras, feeds them into a persistent GPU
 * log-odds voxel grid, and publishes a robot-centric 3-D voxel occupancy observation
 * (32×32×32 binary crop, {@value CUDAGPUVoxelGrid#VOXEL_CROP_SIZE} voxels, bit-packed to
 * {@value us.ihmc.communication.VoxelOccupancyPacking#PACKED_FLOAT_COUNT} floats) on
 * {@link PerceptionAPI#VOXEL_OCCUPANCY} for the fall-recovery RL policy.
 */
public class DualCameraVoxelGridThread extends RepeatingTaskThread
{
   // Stepping (belly) camera dead band — discard points closer than 30 cm to avoid self-hits.
   private static final float STEPPING_DEPTH_MIN_M     = 0.30f;
   private static final float STEPPING_DEPTH_MAX_M     = 10.0f;
   private static final float EXPERIMENTAL_DEPTH_MIN_M = 0.10f;
   private static final float EXPERIMENTAL_DEPTH_MAX_M = 8.0f;

   private static final int    DILATION_MIN_NEIGHBOURS  = 2;
   private static final int    DILATION_FILL_LOG_ODDS   = 300;
   private static final double PUBLISH_FREQUENCY_HZ     = 30.0;

   // Distance-dependent measurement-confidence model (see depthConfidenceWeight in PerceptionUtils.cu).
   // ZED X Mini depth accuracy degrades with range, so each hit's log-odds is weighted by
   // (referenceRange / range) ^ falloffExponent: points at or nearer than the reference range get
   // full weight, farther points count progressively less. Because the belly-mounted stepping
   // camera observes the ground at shorter range than the head camera, this makes the height map
   // trust the stepping (pelvis) camera more than the experimental (head) camera automatically.
   private static final float CONFIDENCE_REFERENCE_RANGE_M = 2.0f;
   private static final float CONFIDENCE_FALLOFF_EXPONENT  = 2.0f;
   // Per-camera trust multipliers on top of the range effect; lower the head camera to bias further.
   private static final float STEPPING_CAMERA_TRUST     = 1.0f;
   private static final float EXPERIMENTAL_CAMERA_TRUST = 1.0f;

   private final BlockingQueue<RawImage> steppingDepthQueue     = new LinkedBlockingQueue<>(4);
   private final BlockingQueue<RawImage> experimentalDepthQueue = new LinkedBlockingQueue<>(4);
   // Optional per-pixel ZED confidence maps, aligned with the depth frames from the same grab.
   // When empty, points are weighted by range only.
   private final BlockingQueue<RawImage> steppingConfidenceQueue     = new LinkedBlockingQueue<>(4);
   private final BlockingQueue<RawImage> experimentalConfidenceQueue = new LinkedBlockingQueue<>(4);

   private final CUDAPointCloudExtractor steppingDepthExtractor     = new CUDAPointCloudExtractor();
   private final CUDAPointCloudExtractor experimentalDepthExtractor = new CUDAPointCloudExtractor();
   private final CUDAGPUVoxelGrid voxelGrid;

   private final ROS2SyncedRobotModel syncedRobot;
   private final RigidBodyTransform pelvisToWorld = new RigidBodyTransform();

   private final ROS2Publisher<Float32MultiArrayHack> voxelOccupancyPublisher;
   private final Float32MultiArrayHack voxelOccupancyMessage = new Float32MultiArrayHack();
   private final float[] voxelOccupancyBuffer = new float[CUDAGPUVoxelGrid.VOXEL_CROP_SIZE];
   // Bit-packed wire form (32 voxels/float) so the 32768-voxel crop fits the message's 1024-float bound.
   private final float[] voxelOccupancyPacked = new float[VoxelOccupancyPacking.PACKED_FLOAT_COUNT];

   // Optional whole-grid occupied-voxel snapshot for RDX debug visualisation (positions + probabilities).
   private volatile boolean readbackOccupiedVoxelsForViz = false;
   private volatile CUDAGPUVoxelGrid.OccupiedVoxels latestOccupiedVoxels =
         new CUDAGPUVoxelGrid.OccupiedVoxels(new float[0], new float[0], 0);

   public DualCameraVoxelGridThread(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this(ros2Node, syncedRobot, 200, 200, 100, 0.05f);
   }

   public DualCameraVoxelGridThread(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot,
                                    int nx, int ny, int nz, float resolution)
   {
      super(DualCameraVoxelGridThread.class.getSimpleName());

      this.syncedRobot = syncedRobot;
      this.voxelGrid   = new CUDAGPUVoxelGrid(nx, ny, nz, resolution);

      steppingDepthExtractor.setConfidenceModel(CONFIDENCE_REFERENCE_RANGE_M, CONFIDENCE_FALLOFF_EXPONENT, STEPPING_CAMERA_TRUST);
      experimentalDepthExtractor.setConfidenceModel(CONFIDENCE_REFERENCE_RANGE_M, CONFIDENCE_FALLOFF_EXPONENT, EXPERIMENTAL_CAMERA_TRUST);

      voxelOccupancyPublisher = ros2Node.createPublisher(PerceptionAPI.VOXEL_OCCUPANCY);

      setFrequencyLimit(PUBLISH_FREQUENCY_HZ);
   }

   @Override
   protected void runTask()
   {
      try
      {
         // Each camera is processed independently — the grid updates with whatever arrived.
         // This allows single-camera operation when one queue is not registered.
         RawImage steppingDepth     = steppingDepthQueue.poll(100, TimeUnit.MILLISECONDS);
         RawImage experimentalDepth = experimentalDepthQueue.poll(100, TimeUnit.MILLISECONDS);

         boolean anyFrame = steppingDepth != null || experimentalDepth != null;
         if (!anyFrame)
            return;

         if (steppingDepth != null)
         {
            // Confidence map for this grab (if the stream is registered) rides a parallel queue.
            RawImage steppingConfidence = steppingConfidenceQueue.poll();
            int count = steppingDepthExtractor.extractToGpu(steppingDepth, steppingConfidence, STEPPING_DEPTH_MIN_M, STEPPING_DEPTH_MAX_M);
            cudaStreamSynchronize(steppingDepthExtractor.getStream());
            if (count > 0)
               voxelGrid.updateHits(steppingDepthExtractor.getGpuPointCloud(),
                                    steppingDepthExtractor.getGpuPointConfidence(), count);
         }

         if (experimentalDepth != null)
         {
            RawImage experimentalConfidence = experimentalConfidenceQueue.poll();
            int count = experimentalDepthExtractor.extractToGpu(experimentalDepth, experimentalConfidence, EXPERIMENTAL_DEPTH_MIN_M, EXPERIMENTAL_DEPTH_MAX_M);
            cudaStreamSynchronize(experimentalDepthExtractor.getStream());
            if (count > 0)
               voxelGrid.updateHits(experimentalDepthExtractor.getGpuPointCloud(),
                                    experimentalDepthExtractor.getGpuPointConfidence(), count);
         }

         voxelGrid.dilate(DILATION_MIN_NEIGHBOURS, DILATION_FILL_LOG_ODDS);

         syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToDesiredFrame(pelvisToWorld, ReferenceFrame.getWorldFrame());
         voxelGrid.extractVoxelObservation(pelvisToWorld, voxelOccupancyBuffer, 0);

         VoxelOccupancyPacking.pack(voxelOccupancyBuffer, voxelOccupancyPacked);
         voxelOccupancyMessage.getData().clear();
         for (float packed : voxelOccupancyPacked)
            voxelOccupancyMessage.getData().add(packed);
         voxelOccupancyPublisher.publish(voxelOccupancyMessage);

         // Optional whole-grid occupied-voxel snapshot for RDX debug visualisation (off by default
         // since the readback copies every occupied voxel to the CPU each frame).
         if (readbackOccupiedVoxelsForViz)
            latestOccupiedVoxels = voxelGrid.extractOccupiedVoxels();
      }
      catch (InterruptedException ignored)
      {
      }
      catch (Exception e)
      {
         LogTools.error("Voxel height scan error: {}", e.getMessage());
      }
   }

   /**
    * Centres the voxel grid on the robot's current pelvis position.
    * Call once after the robot's state is first available.
    */
   public void centreGridOnCurrentPose()
   {
      FramePoint3D pelvis = new FramePoint3D(syncedRobot.getReferenceFrames().getPelvisFrame());
      pelvis.changeFrame(ReferenceFrame.getWorldFrame());
      voxelGrid.centreOnRobot((float) pelvis.getX(), (float) pelvis.getY(), (float) pelvis.getZ());
   }

   /** Clears the voxel grid. Call at episode start. */
   public void resetGrid() { voxelGrid.clear(); }

   /** Enable/disable the per-frame whole-grid occupied-voxel readback used by the RDX debug viewer. */
   public void setReadbackOccupiedVoxelsForViz(boolean enabled) { readbackOccupiedVoxelsForViz = enabled; }

   /**
    * Latest whole-grid occupied-voxel snapshot (world positions + occupancy probabilities) for
    * visualisation. Empty unless {@link #setReadbackOccupiedVoxelsForViz} is enabled. Thread-safe to read.
    */
   public CUDAGPUVoxelGrid.OccupiedVoxels getLatestOccupiedVoxels() { return latestOccupiedVoxels; }

   /** Register with the stepping camera depth image stream. */
   public BlockingQueue<RawImage> getSteppingDepthQueue()     { return steppingDepthQueue; }
   /** Register with the experimental camera depth image stream. */
   public BlockingQueue<RawImage> getExperimentalDepthQueue() { return experimentalDepthQueue; }
   /** Register with the stepping camera confidence map stream (optional; enables confidence-weighted hits). */
   public BlockingQueue<RawImage> getSteppingConfidenceQueue()     { return steppingConfidenceQueue; }
   /** Register with the experimental camera confidence map stream (optional; enables confidence-weighted hits). */
   public BlockingQueue<RawImage> getExperimentalConfidenceQueue() { return experimentalConfidenceQueue; }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();
      steppingDepthExtractor.close();
      experimentalDepthExtractor.close();
      voxelGrid.close();
   }
}
