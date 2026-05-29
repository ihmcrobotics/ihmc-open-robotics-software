package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.ImageMessage;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXFusedVoxelMapVisualizer;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.List;
import java.util.Set;

/**
 * Subscribes to depth <em>and</em> left-colour topics for both cameras and feeds decoded
 * {@link RawImage} objects into {@link RDXFusedVoxelMapVisualizer} for CUDA fusion,
 * confidence-weighted voxelisation, and camera-coloured rendering.
 */
public class RDXROS2FusedVoxelMapVisualizer extends RDXROS2MultiTopicVisualizer
{
   private final ROS2Node ros2Node;

   // ── Stepping camera (belly, 4 mm) ─────────────────────────────────────────
   private final ROS2Topic<ImageMessage> steppingDepthTopic;
   private final ROS2Topic<ImageMessage> steppingColorTopic;
   private ROS2Subscription<ImageMessage> steppingDepthSub;
   private ROS2Subscription<ImageMessage> steppingColorSub;
   private final ImageMessageDecoder steppingDepthDecoder = new ImageMessageDecoder();
   private final ImageMessageDecoder steppingColorDecoder = new ImageMessageDecoder();
   private final RDXMessageSizeReadout steppingDepthSize   = new RDXMessageSizeReadout();
   private final RDXMessageSizeReadout steppingColorSize   = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot steppingDepthDiscontinuity = new RDXSequenceDiscontinuityPlot();

   // ── Experimental camera (head, 2.2 mm) ───────────────────────────────────
   private final ROS2Topic<ImageMessage> experimentalDepthTopic;
   private final ROS2Topic<ImageMessage> experimentalColorTopic;
   private ROS2Subscription<ImageMessage> experimentalDepthSub;
   private ROS2Subscription<ImageMessage> experimentalColorSub;
   private final ImageMessageDecoder experimentalDepthDecoder = new ImageMessageDecoder();
   private final ImageMessageDecoder experimentalColorDecoder = new ImageMessageDecoder();
   private final RDXMessageSizeReadout experimentalDepthSize   = new RDXMessageSizeReadout();
   private final RDXMessageSizeReadout experimentalColorSize   = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot experimentalDepthDiscontinuity = new RDXSequenceDiscontinuityPlot();

   // ── Heartbeats ────────────────────────────────────────────────────────────
   private ROS2Heartbeat steppingHeartbeat;
   private ROS2Heartbeat experimentalHeartbeat;

   // ── Core fused visualiser ─────────────────────────────────────────────────
   private final RDXFusedVoxelMapVisualizer fusedVisualizer;

   public RDXROS2FusedVoxelMapVisualizer(String title,
                                          ROS2Node ros2Node,
                                          ROS2Topic<ImageMessage> steppingDepthTopic,
                                          ROS2Topic<ImageMessage> steppingColorTopic,
                                          ROS2Topic<ImageMessage> experimentalDepthTopic,
                                          ROS2Topic<ImageMessage> experimentalColorTopic)
   {
      super(title);
      this.ros2Node               = ros2Node;
      this.steppingDepthTopic     = steppingDepthTopic;
      this.steppingColorTopic     = steppingColorTopic;
      this.experimentalDepthTopic = experimentalDepthTopic;
      this.experimentalColorTopic = experimentalColorTopic;

      fusedVisualizer = new RDXFusedVoxelMapVisualizer(title);

      addActivenessChangeCallback(active -> { if (active) subscribe(); else unsubscribe(); });
   }

   public void createSteppingHeartbeat(ROS2Node node, ROS2Topic<Empty> topic)
   {
      steppingHeartbeat = new ROS2Heartbeat(node, topic);
   }

   public void createExperimentalHeartbeat(ROS2Node node, ROS2Topic<Empty> topic)
   {
      experimentalHeartbeat = new ROS2Heartbeat(node, topic);
   }

   @Override
   public void updateHeartbeat()
   {
      if (steppingHeartbeat != null)     steppingHeartbeat.setAlive(isActive());
      if (experimentalHeartbeat != null) experimentalHeartbeat.setAlive(isActive());
   }

   @Override
   public void update()
   {
      super.update();
      fusedVisualizer.update();
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return List.of(steppingDepthTopic, steppingColorTopic,
                     experimentalDepthTopic, experimentalColorTopic);
   }

   @Override
   public void renderImGuiWidgets()
   {
      steppingDepthSize.renderImGuiWidgets();
      steppingColorSize.renderImGuiWidgets();
      steppingDepthDiscontinuity.renderImGuiWidgets();
      experimentalDepthSize.renderImGuiWidgets();
      experimentalColorSize.renderImGuiWidgets();
      experimentalDepthDiscontinuity.renderImGuiWidgets();
      fusedVisualizer.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool,
                              Set<RDXSceneLevel> sceneLevels)
   {
      fusedVisualizer.getRenderables(renderables, pool, sceneLevels);
   }

   @Override
   public void destroy()
   {
      super.destroy();
      unsubscribe();
      if (steppingHeartbeat != null)     steppingHeartbeat.destroy();
      if (experimentalHeartbeat != null) experimentalHeartbeat.destroy();
      fusedVisualizer.destroy();
      steppingDepthDecoder.destroy();
      steppingColorDecoder.destroy();
      experimentalDepthDecoder.destroy();
      experimentalColorDecoder.destroy();
   }

   // ── Subscriptions ─────────────────────────────────────────────────────────

   private void subscribe()
   {
      steppingDepthSub     = ros2Node.createSubscription2(steppingDepthTopic,     this::onSteppingDepth);
      steppingColorSub     = ros2Node.createSubscription2(steppingColorTopic,     this::onSteppingColor);
      experimentalDepthSub = ros2Node.createSubscription2(experimentalDepthTopic, this::onExperimentalDepth);
      experimentalColorSub = ros2Node.createSubscription2(experimentalColorTopic, this::onExperimentalColor);
   }

   private void unsubscribe()
   {
      removeSub(steppingDepthSub);     steppingDepthSub = null;
      removeSub(steppingColorSub);     steppingColorSub = null;
      removeSub(experimentalDepthSub); experimentalDepthSub = null;
      removeSub(experimentalColorSub); experimentalColorSub = null;
   }

   private static void removeSub(ROS2Subscription<?> sub)
   {
      if (sub != null) sub.remove();
   }

   // ── Message callbacks ─────────────────────────────────────────────────────

   private void onSteppingDepth(ImageMessage msg)
   {
      if (msg.getData().isEmpty()) return;
      RawImage img = steppingDepthDecoder.decodeMessageCPU(msg);
      fusedVisualizer.setSteppingDepthImage(img);
      getFrequency(steppingDepthTopic).ping();
      steppingDepthSize.update(msg.getData().size());
      steppingDepthDiscontinuity.update(msg.getSequenceNumber());
      img.release();
   }

   private void onSteppingColor(ImageMessage msg)
   {
      if (msg.getData().isEmpty()) return;
      RawImage img = steppingColorDecoder.decodeMessageCPU(msg);
      fusedVisualizer.setSteppingColorImage(img);
      getFrequency(steppingColorTopic).ping();
      steppingColorSize.update(msg.getData().size());
      img.release();
   }

   private void onExperimentalDepth(ImageMessage msg)
   {
      if (msg.getData().isEmpty()) return;
      RawImage img = experimentalDepthDecoder.decodeMessageCPU(msg);
      fusedVisualizer.setExperimentalDepthImage(img);
      getFrequency(experimentalDepthTopic).ping();
      experimentalDepthSize.update(msg.getData().size());
      experimentalDepthDiscontinuity.update(msg.getSequenceNumber());
      img.release();
   }

   private void onExperimentalColor(ImageMessage msg)
   {
      if (msg.getData().isEmpty()) return;
      RawImage img = experimentalColorDecoder.decodeMessageCPU(msg);
      fusedVisualizer.setExperimentalColorImage(img);
      getFrequency(experimentalColorTopic).ping();
      experimentalColorSize.update(msg.getData().size());
      img.release();
   }
}
