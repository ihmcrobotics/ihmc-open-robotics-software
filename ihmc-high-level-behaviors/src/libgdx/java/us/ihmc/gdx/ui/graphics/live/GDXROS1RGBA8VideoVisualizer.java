package us.ihmc.gdx.ui.graphics.live;

import imgui.internal.ImGui;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1VisualizerInterface;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.ZeroCopySwapReference;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1RGBA8VideoVisualizer extends ImGuiGDXVisualizer implements ImGuiGDXROS1VisualizerInterface
{
   private AbstractRosTopicSubscriber<Image> subscriber;
   private final String topic;
   private boolean currentlySubscribed = false;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final ImGuiVideoPanel videoPanel;
   private final ZeroCopySwapReference<GDXROS1RGBA8VideoVisualizerData> dataSwapReferenceManager
         = new ZeroCopySwapReference<>(GDXROS1RGBA8VideoVisualizerData::new);

   public GDXROS1RGBA8VideoVisualizer(String title, String topic)
   {
      super(title + " (ROS 1)");
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      boolean flipY = false;
      videoPanel = new ImGuiVideoPanel(ImGuiTools.uniqueLabel(this, topic), flipY);
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      subscriber = new AbstractRosTopicSubscriber<Image>(Image._TYPE)
      {
         @Override
         public void onNewMessage(Image image)
         {
            doReceiveMessageOnThread(() -> processIncomingMessageOnThread(image));
         }
      };
      ros1Node.attachSubscriber(topic, subscriber);
   }

   protected void doReceiveMessageOnThread(Runnable receiveMessageOnThread)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(receiveMessageOnThread);
      }
   }

   private void processIncomingMessageOnThread(Image image)
   {
      dataSwapReferenceManager.accessOnLowPriorityThread(data ->
      {
         data.updateOnMessageProcessingThread(image);
      });
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         dataSwapReferenceManager.accessOnLowPriorityThread(data ->
         {
            data.updateOnUIThread(videoPanel);
         });
      }
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic);
      frequencyPlot.renderImGuiWidgets();
   }

   @Override
   public ImGuiVideoPanel getPanel()
   {
      return videoPanel;
   }

   @Override
   public void updateSubscribers(RosNodeInterface ros1Node)
   {
      boolean active = isActive();
      if (active != currentlySubscribed)
      {
         if (active)
         {
            subscribe(ros1Node);
         }
         else
         {
            unsubscribe(ros1Node);
         }
      }
      currentlySubscribed = active;
   }
}
