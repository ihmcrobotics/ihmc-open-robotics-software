package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.javacpp.BytePointer;
import sensor_msgs.msg.dds.Image;
import us.ihmc.log.LogTools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import static org.bytedeco.opencv.global.opencv_core.CV_32FC1;
import static org.bytedeco.opencv.global.opencv_core.CV_8UC3;
import static org.bytedeco.opencv.global.opencv_core.CV_8UC4;

/**
 * RDX visualizer for standard sensor_msgs/Image topics (rgb8, bgr8, rgba8, 32FC1, jpeg).
 *
 * Used to display Isaac Sim camera feeds published by alex_isaac_sim_dds.py with
 * {@code --enable_cameras}.  Subscribes to a {@code ROS2Topic<Image>} and decodes
 * the raw pixel data based on the {@code encoding} field.
 *
 * Supported encodings:
 *   "rgb8"   — 3-channel uint8, RGB order
 *   "bgr8"   — 3-channel uint8, BGR order
 *   "rgba8"  — 4-channel uint8, RGBA order
 *   "32FC1"  — single-channel float32 depth (normalised to 8-bit for display)
 *   "jpeg"   — JPEG-compressed bytes (step == 0 convention from alex_isaac_sim_dds.py)
 */
public class RDXROS2SensorMsgsImageVisualizer extends RDXROS2ImageVisualizer<Image>
{
   private final ROS2Topic<Image> topic;
   private final ROS2Node ros2Node;
   private ROS2Subscription<Image> subscription;

   // Reused across callbacks — only touched on the image-update thread
   private final Mat decodedMat = new Mat();
   private final Mat normalizedMat = new Mat();

   public RDXROS2SensorMsgsImageVisualizer(String title, ROS2Node ros2Node, ROS2Topic<Image> topic)
   {
      super(title, topic.getName(), false);
      this.topic = topic;
      this.ros2Node = ros2Node;

      addActivenessChangeCallback(isActive ->
      {
         if (isActive && subscription == null)
            subscribe();
         else if (!isActive && subscription != null)
            unsubscribe();
      });
   }

   private void subscribe()
   {
      if (subscription != null)
         subscription.remove();
      LogTools.info("Subscribing to {}", topic.getName());
      subscription = ros2Node.createSubscription2(topic, this::onMessage);
   }

   private void unsubscribe()
   {
      if (subscription != null)
         subscription.remove();
      subscription = null;
   }

   private volatile int messageCount = 0;

   private void onMessage(Image message)
   {
      if (messageCount++ < 3)
         LogTools.info("onMessage #{} on {} info={}", messageCount, topic.getName(),
                       message.getWidth() + "x" + message.getHeight() + " " + message.getEncodingAsString());
      getFrequency().ping();

      // Copy primitive fields — message object is reused by the subscription layer
      final int width    = (int) message.getWidth();
      final int height   = (int) message.getHeight();
      final String encoding = message.getEncodingAsString();
      final byte[] data  = message.getData().copyArray();

      submitImageUpdate(imageVisualizer ->
      {
         if (width <= 0 || height <= 0 || data == null || data.length == 0)
            return;

         boolean decoded = decode(data, width, height, encoding);
         if (!decoded)
         {
            LogTools.warn("Failed to decode image: encoding={} dims={} dataLen={}", encoding, width + "x" + height, data.length);
            return;
         }

         PixelFormat pixelFormat = encodingToPixelFormat(encoding);
         imageVisualizer.setImage(decodedMat, pixelFormat);
      });
   }

   /**
    * Decode raw image bytes into {@code decodedMat} based on the ROS encoding string.
    *
    * @return true if decoding succeeded, false to skip this frame.
    */
   private boolean decode(byte[] data, int width, int height, String encoding)
   {
      switch (encoding)
      {
         case "rgb8":
         {
            // Raw RGB bytes — wrap directly into a Mat and convert to BGR for OpenCV
            BytePointer ptr = new BytePointer(data);
            Mat raw = new Mat(height, width, CV_8UC3, ptr);
            opencv_imgproc.cvtColor(raw, decodedMat, opencv_imgproc.COLOR_RGB2BGR);
            ptr.deallocate();
            return true;
         }
         case "bgr8":
         {
            BytePointer ptr = new BytePointer(data);
            new Mat(height, width, CV_8UC3, ptr).copyTo(decodedMat);
            ptr.deallocate();
            return true;
         }
         case "rgba8":
         {
            BytePointer ptr = new BytePointer(data);
            Mat raw = new Mat(height, width, CV_8UC4, ptr);
            opencv_imgproc.cvtColor(raw, decodedMat, opencv_imgproc.COLOR_RGBA2BGR);
            ptr.deallocate();
            return true;
         }
         case "32FC1":
         {
            // Float depth in metres — normalise to 0–255 for display
            BytePointer ptr = new BytePointer(data);
            Mat floatMat = new Mat(height, width, CV_32FC1, ptr);
            org.bytedeco.opencv.global.opencv_core.normalize(floatMat, normalizedMat, 0, 255,
                  org.bytedeco.opencv.global.opencv_core.NORM_MINMAX, org.bytedeco.opencv.global.opencv_core.CV_8U, new Mat());
            opencv_imgproc.cvtColor(normalizedMat, decodedMat, opencv_imgproc.COLOR_GRAY2BGR);
            ptr.deallocate();
            return true;
         }
         case "jpeg":
         {
            // JPEG-compressed bytes — decode with OpenCV imdecode
            BytePointer ptr = new BytePointer(data);
            Mat buf = new Mat(1, data.length, org.bytedeco.opencv.global.opencv_core.CV_8UC1, ptr);
            opencv_imgcodecs.imdecode(buf, opencv_imgcodecs.IMREAD_COLOR, decodedMat);
            ptr.deallocate();
            return !decodedMat.empty();
         }
         default:
            return false;
      }
   }

   private static PixelFormat encodingToPixelFormat(String encoding)
   {
      return switch (encoding)
      {
         case "rgb8"  -> PixelFormat.RGB8;
         case "rgba8" -> PixelFormat.RGB8;   // displayed as RGB after RGBA→BGR conversion
         case "32FC1" -> PixelFormat.BGR8;   // depth rendered as grey BGR
         case "jpeg"  -> PixelFormat.BGR8;   // imdecode returns BGR
         default      -> PixelFormat.BGR8;   // bgr8 and fallback
      };
   }

   @Override
   public void destroy()
   {
      super.destroy();
      unsubscribe();
      decodedMat.deallocate();
      normalizedMat.deallocate();
   }

   @Override
   public ROS2Topic<Image> getTopic()
   {
      return topic;
   }
}
