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
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXVoxelMapRenderer;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXROS2VoxelMapVisualizer extends RDXROS2SingleTopicVisualizer<VoxelMapMessage>
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VoxelMapMessage> topic;
   private ROS2Subscription<VoxelMapMessage> subscription;

   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXVoxelMapRenderer renderer = new RDXVoxelMapRenderer();
   private int rendererMaxVoxels = 0;

   // New maps arrive on the subscription thread; the GL thread swaps via getAndSet(null)
   private final AtomicReference<VoxelMap> pendingVoxelMap = new AtomicReference<>();

   public RDXROS2VoxelMapVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VoxelMapMessage> topic)
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

      // (Re)create renderer only when voxel count grows — typically happens once on first message
      int voxelCount = map.getVoxelCount();
      if (voxelCount > rendererMaxVoxels)
      {
         renderer.create(voxelCount);
         rendererMaxVoxels = voxelCount;
      }

      renderer.update(map);
      map.close(); // CPU data transferred to GL vertex buffer; native memory no longer needed
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

      // Discard any unprocessed map from the previous callback
      VoxelMap old = pendingVoxelMap.getAndSet(newMap);
      if (old != null)
         old.close();

      getFrequency().ping();
      messageSizeReadout.update(voxelCount * Float.BYTES);
   }
}
