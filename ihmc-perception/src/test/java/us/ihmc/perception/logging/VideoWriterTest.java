package us.ihmc.perception.logging;

import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.log.LogTools;

public class VideoWriterTest
{
   public VideoWriterTest()
   {


      VideoWriter videoWriter = new VideoWriter();
      int fourcc = VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G');
      boolean success = videoWriter.open("./test.mp4", fourcc, 30.0, new Size(640, 480), true);
      LogTools.info(success);
      videoWriter.release();
   }

   public static void main(String[] args)
   {
      new VideoWriterTest();
   }
}
