package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1VisualizerInterface;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.ByteBuffer;

public class GDXROS1RGBA8VideoVisualizer extends ImGuiGDXVisualizer implements ImGuiGDXROS1VisualizerInterface
{
   private AbstractRosTopicSubscriber<Image> subscriber;
   private final String topic;
   private boolean currentlySubscribed = false;
   private Mat rgba8Mat;
   private boolean needNewTexture = false;
   private BytePointer rgba8888BytePointer;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private Pixmap pixmap;
   private Texture texture;
   private final ImGuiVideoPanel videoPanel;

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
      ChannelBuffer nettyChannelBuffer = image.getData();

      int imageWidth = image.getWidth();
      int imageHeight = image.getHeight();

      updateImageDimensions(imageWidth, imageHeight);
      ByteBuffer inputByteBuffer = RosTools.sliceNettyBuffer(nettyChannelBuffer);
      for (int i = 0; i < inputByteBuffer.limit(); i++)
      {
         rgba8888BytePointer.put(i, inputByteBuffer.get());
      }
   }

   protected void updateImageDimensions(int imageWidth, int imageHeight)
   {
      if (rgba8Mat == null || pixmap.getWidth() != imageWidth || pixmap.getHeight() != imageHeight)
      {
         if (pixmap != null)
         {
            pixmap.dispose();
         }

         pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
         rgba8888BytePointer = new BytePointer(pixmap.getPixels());
         rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, rgba8888BytePointer);
         needNewTexture = true;
      }
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         synchronized (this)
         {
            if (rgba8Mat != null)
            {
               if (texture == null || needNewTexture)
               {
                  needNewTexture = false;
                  if (texture != null)
                  {
                     texture.dispose();
                  }

                  texture = new Texture(new PixmapTextureData(pixmap, null, false, false));
                  videoPanel.setTexture(texture);
               }

               texture.draw(pixmap, 0, 0);
            }
         }
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
