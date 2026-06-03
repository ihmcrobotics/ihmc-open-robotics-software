package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import perception_msgs.msg.dds.Float32MultiArrayHack;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.VoxelOccupancyPacking;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.cuda.CUDAGPUVoxelGrid;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Renders the robot-centric 3-D voxel occupancy observation published on
 * {@link PerceptionAPI#VOXEL_OCCUPANCY} — i.e. exactly what the fall-recovery RL policy sees.
 *
 * <p>The message is a 32×32×40 binary occupancy crop (z-as-channel) in a yaw-aligned frame centred
 * on the robot. Each occupied cell is drawn as a point at its cell centre in that robot-local frame,
 * height-coloured. Mount this under the robot's pelvis frame in the UI to overlay it on the scene.
 */
public class RDXROS2VoxelOccupancyVisualizer extends RDXVisualizer
{
   private static final int   NX  = CUDAGPUVoxelGrid.CROP_NX;
   private static final int   NY  = CUDAGPUVoxelGrid.CROP_NY;
   private static final int   NZ  = CUDAGPUVoxelGrid.CROP_NZ;
   private static final float RES = CUDAGPUVoxelGrid.CROP_RESOLUTION;
   private static final int   CROP_SIZE = NX * NY * NZ;

   private final ROS2Node ros2Node;
   private final ROS2Topic<Float32MultiArrayHack> topic;
   private ROS2Subscription<Float32MultiArrayHack> subscription;

   private final Notification newMessage = new Notification();
   private float[] latestOccupancy = null; // guarded by this

   private final RDXPointCloudRenderer renderer = new RDXPointCloudRenderer();
   private boolean rendererCreated = false;
   private final List<Point3D32> occupiedCells = new ArrayList<>();

   // Hide crop voxels whose occupancy probability is below this threshold.
   private final imgui.type.ImFloat probabilityThreshold = new imgui.type.ImFloat(0.5f);

   public RDXROS2VoxelOccupancyVisualizer(String title, ROS2Node ros2Node)
   {
      this(title, ros2Node, PerceptionAPI.VOXEL_OCCUPANCY);
   }

   public RDXROS2VoxelOccupancyVisualizer(String title, ROS2Node ros2Node, ROS2Topic<Float32MultiArrayHack> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      setSceneLevels(RDXSceneLevel.MODEL);
      addActivenessChangeCallback(active -> { if (active) subscribe(); else unsubscribe(); });
   }

   private void subscribe()
   {
      if (subscription == null)
         subscription = ros2Node.createSubscription2(topic, this::onMessage);
   }

   private void unsubscribe()
   {
      if (subscription != null)
      {
         subscription.remove();
         subscription = null;
      }
   }

   private void onMessage(Float32MultiArrayHack message)
   {
      IDLSequence.Float data = message.getData();
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

      occupiedCells.clear();
      float halfX = 0.5f * (NX - 1);
      float halfY = 0.5f * (NY - 1);
      float halfZ = 0.5f * (NZ - 1);
      for (int cz = 0; cz < NZ; cz++)
         for (int cy = 0; cy < NY; cy++)
            for (int cx = 0; cx < NX; cx++)
               if (occupancy[cz * (NY * NX) + cy * NX + cx] >= probabilityThreshold.get())
                  occupiedCells.add(new Point3D32((cx - halfX) * RES, (cy - halfY) * RES, (cz - halfZ) * RES));

      renderer.updateMesh(occupiedCells);
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.sliderFloat(labels.get("Probability Threshold"), probabilityThreshold.getData(), 0.5f, 1.0f);
      ImGui.text("Voxels shown: " + occupiedCells.size());
      ImGui.text(String.format("Crop: %d x %d x %d @ %.0f cm (robot frame)", NX, NY, NZ, RES * 100.0f));
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
