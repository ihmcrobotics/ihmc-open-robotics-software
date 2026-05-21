package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.ImageMessage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.List;
import java.util.Set;

public class RDXROS2ColoredPointCloudVisualizer extends RDXROS2MultiTopicVisualizer
{
   private final ROS2Node ros2Node;

   private ROS2Topic<ImageMessage> depthTopic;
   private ROS2Subscription<ImageMessage> depthSubscription;
   private final RDXMessageSizeReadout depthMessageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot depthDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();

   private ROS2Topic<ImageMessage> colorTopic;
   private ROS2Subscription<ImageMessage> colorSubscription;
   private final RDXMessageSizeReadout colorMessageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot colorDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();

   private final ImageMessageDecoder depthMessageDecoder = new ImageMessageDecoder();
   private final ImageMessageDecoder colorMessageDecoder = new ImageMessageDecoder();

   private final RDXRawImagePointCloudVisualizer visualizer;

   public RDXROS2ColoredPointCloudVisualizer(String title, ROS2Node ros2Node, ROS2Topic<ImageMessage> depthTopic, ROS2Topic<ImageMessage> colorTopic)
   {
      super(title);

      this.ros2Node = ros2Node;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;

      visualizer = new RDXRawImagePointCloudVisualizer(title);

      addActivenessChangeCallback(isActive ->
      {
         if (isActive)
            subscribe();
         else
            unsubscribe();
      });
   }

   public void changeTopics(ROS2Topic<ImageMessage> depthTopic, ROS2Topic<ImageMessage> colorTopic)
   {
      boolean wasSubscribed = depthSubscription != null;
      unsubscribe();

      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;

      if (wasSubscribed)
         subscribe();
   }

   @Override
   public void update()
   {
      super.update();
      visualizer.update();
   }

   @Override
   public List<ROS2Topic<?>> getTopics()
   {
      return List.of(depthTopic, colorTopic);
   }

   @Override
   public void renderImGuiWidgets()
   {
      depthMessageSizeReadout.renderImGuiWidgets();
      colorMessageSizeReadout.renderImGuiWidgets();

      depthDiscontinuityPlot.renderImGuiWidgets();
      colorDiscontinuityPlot.renderImGuiWidgets();

      visualizer.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      visualizer.getRenderables(renderables, pool, sceneLevels);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      unsubscribe();
      visualizer.destroy();
      depthMessageDecoder.destroy();
      colorMessageDecoder.destroy();
   }

   private void subscribe()
   {
      depthSubscription = ros2Node.createSubscription(depthTopic, reader -> this.onDepthMessageReceived(reader.read()));
      colorSubscription = ros2Node.createSubscription(colorTopic, reader -> this.onColorMessageReceived(reader.read()));
   }

   private void onDepthMessageReceived(ImageMessage message)
   {
      if (message.getData().isEmpty())
         return;

      RawImage depthImage = depthMessageDecoder.decodeMessageCPU(message);
      visualizer.setDepthImage(depthImage);

      getFrequency(depthTopic).ping();
      depthMessageSizeReadout.update(message.getData().size());
      depthDiscontinuityPlot.update(message.getSequenceNumber());

      depthImage.release();
   }

   private void onColorMessageReceived(ImageMessage message)
   {
      if (message.getData().isEmpty())
         return;

      RawImage colorImage = colorMessageDecoder.decodeMessageCPU(message);
      visualizer.setColorImage(colorImage);

      getFrequency(colorTopic).ping();
      colorMessageSizeReadout.update(message.getData().size());
      colorDiscontinuityPlot.update(message.getSequenceNumber());

      colorImage.release();
   }

   private void unsubscribe()
   {
      if (depthSubscription != null)
      {
         ros2Node.destroySubscription(depthSubscription);
         depthSubscription = null;
      }
      if (colorSubscription != null)
      {
         ros2Node.destroySubscription(colorSubscription);
         colorSubscription = null;
      }
   }
}
