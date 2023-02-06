package us.ihmc.rdx.ui.graphics.live.ros1;

import imgui.internal.ImGui;
import sensor_msgs.Image;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXROS1VisualizerInterface;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.GuidedSwapReference;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

@Deprecated
public class RDXROS1RGBA8VideoVisualizer extends RDXVisualizer implements RDXROS1VisualizerInterface
{
   private AbstractRosTopicSubscriber<Image> subscriber;
   private final String topic;
   private boolean currentlySubscribed = false;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final Runnable doReceiveMessageOnThread = this::doReceiveMessageOnThread;
   private final ImGuiVideoPanel videoPanel;
   private final GuidedSwapReference<RDXROS1RGBA8VideoVisualizerData> dataSwapReferenceManager
                                              = new GuidedSwapReference<>(RDXROS1RGBA8VideoVisualizerData::new,
                                                                          this::processOnLowPriorityThread,
                                                                          this::updateImagePanelOnUIThread);
   private volatile Image latestImage;

   public RDXROS1RGBA8VideoVisualizer(String title, String topic)
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
      subscriber = new AbstractRosTopicSubscriber<>(Image._TYPE)
      {
         @Override
         public void onNewMessage(Image image)
         {
            latestImage = image;
            frequencyPlot.recordEvent();
            if (isActive())
            {
               threadQueue.clearQueueAndExecute(doReceiveMessageOnThread);
            }
         }
      };
      ros1Node.attachSubscriber(topic, subscriber);
   }

   protected void doReceiveMessageOnThread()
   {
      dataSwapReferenceManager.accessOnLowPriorityThread();
   }

   private void processOnLowPriorityThread(RDXROS1RGBA8VideoVisualizerData data)
   {
      data.updateOnMessageProcessingThread(latestImage);
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         dataSwapReferenceManager.accessOnLowPriorityThread();
      }
   }

   private void updateImagePanelOnUIThread(RDXROS1RGBA8VideoVisualizerData data)
   {
      data.updateOnUIThread(videoPanel);
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
