package us.ihmc.avatar.logProcessor.leRobot;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;

import static org.bytedeco.opencv.global.opencv_videoio.*;

public class LeRobotDatasetTools
{
   public static List<Path> findLeRobotDatasetSubdirectories(Path startDirectory)
   {
      try (var paths = Files.walk(startDirectory))
      {
         return paths.filter(Files::isDirectory).filter(directory -> Files.exists(directory.resolve("meta/info.json"))).toList();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error searching for dataset directories with meta/info.json", e);
      }
   }

   public static void appendLine(Path path, String line)
   {
      ExceptionTools.handle(() -> Files.writeString(path, line, StandardOpenOption.APPEND), DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public static void extractMP4FromZED(ZEDSVOScrubber scrubber, Path mp4Path, long startTimestamp, long endTimestamp)
   {
      int imageWidth = scrubber.getImageWidth();
      int imageHeight = scrubber.getImageHeight();


      // TODO resize to 853x480 or 640x480

      // Input #0, mov,mp4,m4a,3gp,3g2,mj2, from 'episode_000000.mp4':
      //  Metadata:
      //    major_brand     : isom
      //    minor_version   : 512
      //    compatible_brands: isomav01iso2mp41
      //    encoder         : Lavf61.3.104
      //  Duration: 00:00:30.00, start: 0.000000, bitrate: 1355 kb/s
      //  Stream #0:0[0x1](und): Video: av1 (Main) (av01 / 0x31307661), yuv420p(tv, progressive), 640x480, 1352 kb/s, 50 fps, 50 tbr, 12800 tbn (default)
      //    Metadata:
      //      handler_name    : VideoHandler
      //      vendor_id       : [0][0][0][0]
      //      encoder         : Lavc61.8.100 libsvtav1
      //[libdav1d @ 0x652476360640] libdav1d 1.2.1

      VideoWriter videoWriter = new VideoWriter();

      int fourcc = VideoWriter.fourcc((byte) 'A', (byte) 'V', (byte) '0', (byte) '1');

//      opencv_imgproc.cvtColor(bgrFrame, yuvFrame, cv::COLOR_BGR2YUV_I420);

      Size frameSize = new Size(imageWidth, imageHeight);
      boolean isColor = true;
      double fps = 50.0;
      videoWriter.open(mp4Path.toString(), fourcc, fps, frameSize, isColor);


      frameSize.close();
   }
}
