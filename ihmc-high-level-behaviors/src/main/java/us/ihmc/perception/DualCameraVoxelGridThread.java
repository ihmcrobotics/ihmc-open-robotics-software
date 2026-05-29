package us.ihmc.perception;

import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAGPUVoxelGrid;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Extracts point clouds from both ZED X Mini depth cameras, feeds them into a GPU log-odds voxel grid,
 * and publishes a 17x11 height-scan observation on {@link PerceptionAPI#VOXEL_HEIGHT_SCAN}.
 */
public class DualCameraVoxelGridThread extends RepeatingTaskThread
{
   public static final int   DEFAULT_NX         = 200;
   public static final int   DEFAULT_NY         = 200;
   public static final int   DEFAULT_NZ         = 100;
   public static final float DEFAULT_RESOLUTION = 0.05f;

   private static final float STEPPING_MIN_M     = 0.30f;
   private static final float STEPPING_MAX_M     = 10.0f;
   private static final float EXPERIMENTAL_MIN_M = 0.10f;
   private static final float EXPERIMENTAL_MAX_M = 8.0f;

   // Require 2 occupied XY neighbours to avoid firing on isolated voxels
   private static final int DILATION_MIN_NEIGHBOURS = 2;
   private static final int DILATION_FILL_LOG_ODDS  = 300;

   private final BlockingQueue<RawImage> steppingDepthQueue     = new LinkedBlockingQueue<>(4);
   private final BlockingQueue<RawImage> experimentalDepthQueue = new LinkedBlockingQueue<>(4);

   private final CUDAPointCloudExtractor steppingExtractor     = new CUDAPointCloudExtractor();
   private final CUDAPointCloudExtractor experimentalExtractor = new CUDAPointCloudExtractor();
   private final CUDAGPUVoxelGrid voxelGrid;

   private final ROS2SyncedRobotModel syncedRobot;
   private final RigidBodyTransform baseToWorld = new RigidBodyTransform();

   private final ROS2Publisher<HeightMapMessage> heightScanPublisher;
   private final HeightMapMessage heightScanMessage = new HeightMapMessage();
   private final float[] heightScanBuffer = new float[CUDAGPUVoxelGrid.NUM_SCAN_POINTS];
   private long sequenceNumber = 0;

   public DualCameraVoxelGridThread(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this(ros2Node, syncedRobot, DEFAULT_NX, DEFAULT_NY, DEFAULT_NZ, DEFAULT_RESOLUTION);
   }

   public DualCameraVoxelGridThread(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot,
                                    int nx, int ny, int nz, float resolution)
   {
      super(DualCameraVoxelGridThread.class.getSimpleName());

      this.syncedRobot = syncedRobot;
      this.voxelGrid   = new CUDAGPUVoxelGrid(nx, ny, nz, resolution);

      heightScanPublisher = ros2Node.createPublisher(PerceptionAPI.VOXEL_HEIGHT_SCAN);

      setFrequencyLimit(30.0);
   }

   @Override
   protected void runTask()
   {
      try
      {
         // Poll with timeout so one slow camera doesn't permanently stall the other.
         RawImage steppingDepth     = steppingDepthQueue.poll(100, TimeUnit.MILLISECONDS);
         RawImage experimentalDepth = experimentalDepthQueue.poll(100, TimeUnit.MILLISECONDS);
         if (steppingDepth == null || experimentalDepth == null)
         {
            if (steppingDepth != null)     steppingDepth.release();
            if (experimentalDepth != null) experimentalDepth.release();
            return;
         }

         List<Point3D32> steppingPts     = steppingExtractor.extractPointCloud(steppingDepth);
         List<Point3D32> experimentalPts = experimentalExtractor.extractPointCloud(experimentalDepth);

         voxelGrid.updateHitsFromList(steppingPts);
         voxelGrid.updateHitsFromList(experimentalPts);

         voxelGrid.dilate(DILATION_MIN_NEIGHBOURS, DILATION_FILL_LOG_ODDS);

         ReferenceFrame baseFrame = syncedRobot.getReferenceFrames().getPelvisFrame();
         baseFrame.getTransformToDesiredFrame(baseToWorld, ReferenceFrame.getWorldFrame());

         voxelGrid.extractHeightScan(baseToWorld, heightScanBuffer, 0);

         heightScanMessage.getHeights().clear();
         for (float h : heightScanBuffer)
            heightScanMessage.getHeights().add(h);
         heightScanMessage.setSequenceId(sequenceNumber++);
         heightScanPublisher.publish(heightScanMessage);
      }
      catch (InterruptedException ignored)
      {
      }
      catch (Exception e)
      {
         LogTools.error("DualCameraVoxelGridThread error: {}", e.getMessage());
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
      steppingExtractor.close();
      experimentalExtractor.close();
      voxelGrid.close();
   }
}
