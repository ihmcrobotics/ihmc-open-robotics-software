package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import perception_msgs.Float32MultiArrayHack;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.VoxelOccupancyPacking;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;
import us.ihmc.perception.cuda.CUDAGPUVoxelGrid;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Renders the robot-centric 3-D voxel occupancy observation published on
 * {@link PerceptionAPI#VOXEL_OCCUPANCY} — i.e. exactly what the fall-recovery RL policy sees.
 *
 * <p>The message is a 32×32×32 binary occupancy crop (z-as-channel) in a yaw-aligned frame centred
 * on the robot. The wire message carries no pose (it's robot-relative by design), so this visualizer
 * re-applies the SAME robot-centric, yaw-only transform the crop was extracted with server-side
 * (see {@code CUDAGPUVoxelGrid#extractVoxelObservation}), using the live synced-robot pelvis pose,
 * rather than relying on any scene-graph parenting. Each occupied cell is drawn as a point at its
 * cell centre in world frame, height-coloured.
 */
public class RDXROS2VoxelOccupancyVisualizer extends RDXVisualizer
{
   private static final int   NX  = CUDAGPUVoxelGrid.CROP_NX;
   private static final int   NY  = CUDAGPUVoxelGrid.CROP_NY;
   private static final int   NZ  = CUDAGPUVoxelGrid.CROP_NZ;
   private static final float RES = CUDAGPUVoxelGrid.CROP_RESOLUTION;
   private static final int   CROP_SIZE = NX * NY * NZ;

   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Topic<Float32MultiArrayHack> topic;
   private ROS2Subscription<Float32MultiArrayHack> subscription;
   private final RigidBodyTransform pelvisToWorld = new RigidBodyTransform();

   private final Notification newMessage = new Notification();
   private float[] latestOccupancy = null; // guarded by this

   private final RDXPointCloudRenderer renderer = new RDXPointCloudRenderer();
   private boolean rendererCreated = false;
   private final List<Point3D32> occupiedCells = new ArrayList<>();

   // Hide crop voxels whose occupancy probability is below this threshold.
   private final imgui.type.ImFloat probabilityThreshold = new imgui.type.ImFloat(0.5f);

   public RDXROS2VoxelOccupancyVisualizer(String title, ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this(title, ros2Node, syncedRobot, PerceptionAPI.VOXEL_OCCUPANCY);
   }

   public RDXROS2VoxelOccupancyVisualizer(String title, ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot, ROS2Topic<Float32MultiArrayHack> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
      this.topic = topic;
      setSceneLevels(RDXSceneLevel.MODEL);
      addActivenessChangeCallback(active -> { if (active) subscribe(); else unsubscribe(); });
   }

   private void subscribe()
   {
      if (subscription == null)
         subscription = ros2Node.createSubscriptionSampler(topic, this::onMessage);
   }

   private void unsubscribe()
   {
      if (subscription != null)
      {
         ros2Node.destroySubscription(subscription);
         subscription = null;
      }
   }

   private void onMessage(Float32MultiArrayHack message)
   {
      IDLFloatSequence data = message.getData();
      if (data.size() != VoxelOccupancyPacking.PACKED_FLOAT_COUNT)
         return;

      float[] packed = new float[VoxelOccupancyPacking.PACKED_FLOAT_COUNT];
      for (int i = 0; i < packed.length; i++)
         packed[i] = data.get(i);

      float[] occupancy = new float[VoxelOccupancyPacking.VOXEL_COUNT];
      VoxelOccupancyPacking.unpack(packed, occupancy);

      synchronized (this)
      {
         latestOccupancy = occupancy;
      }
      newMessage.set();
   }

   @Override
   public void create()
   {
      super.create();
      renderer.create(CROP_SIZE);
      renderer.setColoringMethod(AbstractRDXPointCloudRenderer.ColoringMethod.GRADIENT_WORLD_Z);
      renderer.setPointScale(0.6f * RES);
      rendererCreated = true;
   }

   @Override
   public void update()
   {
      super.update();
      if (!rendererCreated || !newMessage.poll())
         return;

      float[] occupancy;
      synchronized (this)
      {
         occupancy = latestOccupancy;
      }
      if (occupancy == null || occupancy.length < CROP_SIZE)
         return;

      // Re-derive the same robot-centric, yaw-only transform the crop was extracted with
      // (CUDAGPUVoxelGrid#extractVoxelObservation): translate by the pelvis position, rotate
      // crop-local X/Y by yaw only (Z stays gravity-aligned even when the robot is prone).
      syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToDesiredFrame(pelvisToWorld, ReferenceFrame.getWorldFrame());
      float robotX = (float) pelvisToWorld.getTranslation().getX();
      float robotY = (float) pelvisToWorld.getTranslation().getY();
      float robotZ = (float) pelvisToWorld.getTranslation().getZ();
      float yaw = (float) pelvisToWorld.getRotation().getYaw();
      float cosYaw = (float) Math.cos(yaw);
      float sinYaw = (float) Math.sin(yaw);

      occupiedCells.clear();
      float halfX = 0.5f * (NX - 1);
      float halfY = 0.5f * (NY - 1);
      float halfZ = 0.5f * (NZ - 1);
      for (int cz = 0; cz < NZ; cz++)
         for (int cy = 0; cy < NY; cy++)
            for (int cx = 0; cx < NX; cx++)
               if (occupancy[cz * (NY * NX) + cy * NX + cx] >= probabilityThreshold.get())
               {
                  float lx = (cx - halfX) * RES;
                  float ly = (cy - halfY) * RES;
                  float lz = (cz - halfZ) * RES;
                  float wx = robotX + cosYaw * lx - sinYaw * ly;
                  float wy = robotY + sinYaw * lx + cosYaw * ly;
                  float wz = robotZ + lz;
                  occupiedCells.add(new Point3D32(wx, wy, wz));
               }

      renderer.updateMesh(occupiedCells);
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.sliderFloat(labels.get("Probability Threshold"), probabilityThreshold.getData(), 0.5f, 1.0f);
      ImGui.text("Voxels shown: " + occupiedCells.size());
      ImGui.text(String.format("Crop: %d x %d x %d @ %.0f cm (robot-centric, drawn in world frame)", NX, NY, NZ, RES * 100.0f));
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (rendererCreated && isActive() && sceneLevelCheck(sceneLevels))
         renderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      if (rendererCreated)
         renderer.dispose();
      super.destroy();
   }
}
