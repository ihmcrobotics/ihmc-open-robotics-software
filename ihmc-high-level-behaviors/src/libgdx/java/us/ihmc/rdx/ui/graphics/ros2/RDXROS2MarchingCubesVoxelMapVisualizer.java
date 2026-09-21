package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.VoxelMapMessage;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.voxelMap.VoxelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMarchingCubesVoxelMapRenderer;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Visualizes a TSDF voxel map as a marching-cubes isosurface extracted at tsdf=0.
 *
 * <p>Subscribes to the same {@link VoxelMapMessage} topic used by the occupancy and TSDF cube
 * visualizers.  On each received message the TSDF float array is handed to
 * {@link RDXMarchingCubesVoxelMapRenderer}, which runs the CPU marching cubes algorithm and
 * uploads the resulting triangle mesh to the GPU for opaque, height-colored rendering.</p>
 */
public class RDXROS2MarchingCubesVoxelMapVisualizer extends RDXROS2SingleTopicVisualizer<VoxelMapMessage>
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VoxelMapMessage> topic;
   private ROS2Subscription<VoxelMapMessage> subscription;

   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXMarchingCubesVoxelMapRenderer renderer = new RDXMarchingCubesVoxelMapRenderer();
   private int rendererMaxVoxels = 0;

   private final AtomicReference<VoxelMap> pendingVoxelMap = new AtomicReference<>();

   public RDXROS2MarchingCubesVoxelMapVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VoxelMapMessage> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;

      addActivenessChangeCallback(isActive ->
      {
         if (isActive)
            subscribe();
         else
            unsubscribe();
      });

      setSceneLevels(RDXSceneLevel.MODEL);
   }

   @Override
   public void update()
   {
      super.update();

      VoxelMap map = pendingVoxelMap.getAndSet(null);
      if (map == null)
         return;

      int voxelCount = map.getVoxelCount();
      if (voxelCount > rendererMaxVoxels)
      {
         renderer.create(voxelCount);
         rendererMaxVoxels = voxelCount;
      }

      renderer.update(map);
      map.close();
   }

   @Override
   public ROS2Topic<VoxelMapMessage> getTopic()
   {
      return topic;
   }

   @Override
   public void renderImGuiWidgets()
   {
      getFrequency().render();
      messageSizeReadout.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
         renderer.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();
      unsubscribe();
      renderer.dispose();
      VoxelMap pending = pendingVoxelMap.getAndSet(null);
      if (pending != null)
         pending.close();
   }

   private void subscribe()
   {
      subscription = ros2Node.createSubscriptionSampler(topic, this::onMessageReceived);
   }

   private void unsubscribe()
   {
      if (subscription != null)
      {
         ros2Node.destroySubscription(subscription);
         subscription = null;
      }
   }

   private void onMessageReceived(VoxelMapMessage message)
   {
      int sizeX = message.getSizeX();
      int sizeY = message.getSizeY();
      int sizeZ = message.getSizeZ();
      int voxelCount = sizeX * sizeY * sizeZ;

      if (voxelCount == 0 || message.getVoxelMapData().size() < voxelCount)
         return;

      VoxelMap newMap = VoxelMap.fromMessage(message);

      VoxelMap old = pendingVoxelMap.getAndSet(newMap);
      if (old != null)
         old.close();

      getFrequency().ping();
      messageSizeReadout.update(voxelCount * Float.BYTES);
   }
}
