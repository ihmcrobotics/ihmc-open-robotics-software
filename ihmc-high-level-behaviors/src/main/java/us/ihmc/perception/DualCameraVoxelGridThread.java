package us.ihmc.perception;

import perception_msgs.msg.dds.Float32MultiArrayHack;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
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
 * Extracts point clouds from both ZED X Mini depth cameras, feeds them into a GPU log-odds voxel grid,
 * and publishes a 17x11 height-scan observation on {@link PerceptionAPI#VOXEL_HEIGHT_SCAN}.
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

   private final BlockingQueue<RawImage> steppingDepthQueue     = new LinkedBlockingQueue<>(4);
   private final BlockingQueue<RawImage> experimentalDepthQueue = new LinkedBlockingQueue<>(4);

   private final CUDAPointCloudExtractor steppingDepthExtractor     = new CUDAPointCloudExtractor();
   private final CUDAPointCloudExtractor experimentalDepthExtractor = new CUDAPointCloudExtractor();
   private final CUDAGPUVoxelGrid voxelGrid;

   private final ROS2SyncedRobotModel syncedRobot;
   private final RigidBodyTransform pelvisToWorld = new RigidBodyTransform();

   private final ROS2Publisher<Float32MultiArrayHack> voxelHeightScanPublisher;
   private final Float32MultiArrayHack voxelHeightScanMessage = new Float32MultiArrayHack();
   private final float[] voxelHeightScanBuffer = new float[CUDAGPUVoxelGrid.NUM_SCAN_POINTS];

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

      voxelHeightScanPublisher = ros2Node.createPublisher(PerceptionAPI.VOXEL_HEIGHT_SCAN);

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
            int count = steppingDepthExtractor.extractToGpu(steppingDepth, STEPPING_DEPTH_MIN_M, STEPPING_DEPTH_MAX_M);
            cudaStreamSynchronize(steppingDepthExtractor.getStream());
            if (count > 0)
               voxelGrid.updateHits(steppingDepthExtractor.getGpuPointCloud(), count);
         }

         if (experimentalDepth != null)
         {
            int count = experimentalDepthExtractor.extractToGpu(experimentalDepth, EXPERIMENTAL_DEPTH_MIN_M, EXPERIMENTAL_DEPTH_MAX_M);
            cudaStreamSynchronize(experimentalDepthExtractor.getStream());
            if (count > 0)
               voxelGrid.updateHits(experimentalDepthExtractor.getGpuPointCloud(), count);
         }

         voxelGrid.dilate(DILATION_MIN_NEIGHBOURS, DILATION_FILL_LOG_ODDS);

         syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToDesiredFrame(pelvisToWorld, ReferenceFrame.getWorldFrame());
         voxelGrid.extractHeightScan(pelvisToWorld, voxelHeightScanBuffer, 0);

         voxelHeightScanMessage.getData().clear();
         for (float h : voxelHeightScanBuffer)
            voxelHeightScanMessage.getData().add(h);
         voxelHeightScanPublisher.publish(voxelHeightScanMessage);
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

   /** Register with the stepping camera depth image stream. */
   public BlockingQueue<RawImage> getSteppingDepthQueue()     { return steppingDepthQueue; }
   /** Register with the experimental camera depth image stream. */
   public BlockingQueue<RawImage> getExperimentalDepthQueue() { return experimentalDepthQueue; }

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
