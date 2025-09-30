package us.ihmc.avatar.logProcessor.leRobot;

import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.FFmpegFrameRecorder;
import org.bytedeco.javacv.Frame;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;

import java.nio.Buffer;
import java.nio.file.Path;

import static us.ihmc.zed.global.zed.*;
import static us.ihmc.zed.global.zed.SL_MEM_CPU;

/**
 * Handles writing the video files for the episodes.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotDatasetVideoWriter
{
   private final RobotSide side;
   private final FFmpegFrameRecorder recorder;

   public LeRobotDatasetVideoWriter(float fps, RobotSide side, Path mp4Path)
   {
      this.side = side;

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

      recorder = new FFmpegFrameRecorder(mp4Path.toString(), 640, 480);
      recorder.setFormat("mp4");
      //            recorder.setVideoCodecName("av1_nvenc");
      //            recorder.setVideoOption("preset", "p7");
      //            recorder.setVideoCodecName("libaom-av1");
      //            recorder.setVideoOption("preset", "10");
      //            recorder.setVideoOption("threads", String.valueOf(Runtime.getRuntime().availableProcessors()));
      recorder.setPixelFormat(avutil.AV_PIX_FMT_YUV420P);
      recorder.setFrameRate(fps);
      recorder.setVideoBitrate(1352000);

      ExceptionTools.handle(() -> recorder.start(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void writeFrame(LeRobotDatasetEpisodeStatistics statistics, ZEDSVOScrubber zedSVOScrubber)
   {
      int imageHeight = zedSVOScrubber.getImageHeight();
      int imageWidth = zedSVOScrubber.getImageWidth();

      Pointer zedColorImageSLMatPointer =
            side == RobotSide.LEFT ? zedSVOScrubber.getLeftColorImageSlMatPointer() : zedSVOScrubber.getRightColorImageSlMatPointer();
      Mat bgra8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                             sl_mat_get_ptr(zedColorImageSLMatPointer, SL_MEM_CPU),
                             sl_mat_get_step_bytes(zedColorImageSLMatPointer, SL_MEM_CPU));

      // Resize smaller for better transformer training
      Size cropSize = new Size(640, 480); // A nice small 4:3 frame
      int scaleWidth = imageWidth * cropSize.height() / imageHeight; // Account for aspect ratio
      Size scaleDownSize = new Size(scaleWidth, cropSize.height());
      Mat resized = new Mat(scaleDownSize, opencv_core.CV_8UC4);
      opencv_imgproc.resize(bgra8Mat, resized, scaleDownSize);
      scaleDownSize.close();

      Point cropOffset = new Point((resized.cols() - cropSize.width()) / 2, 0); // Center crop horizontally
      Rect roi = new Rect(cropOffset, cropSize);
      Mat croppedBgra8 = new Mat(resized, roi);
      resized.close();
      cropOffset.close();

      statistics.submitFrame(side, croppedBgra8); // Wants bgra

      Mat croppedYuv420 = new Mat();
      opencv_imgproc.cvtColor(croppedBgra8, croppedYuv420, opencv_imgproc.COLOR_BGRA2YUV_I420);
      croppedBgra8.close();

      Frame frame = new Frame();
      frame.imageWidth = cropSize.width();
      frame.imageHeight = cropSize.height();
      frame.imageDepth = Frame.DEPTH_UBYTE;
      frame.imageChannels = 1;
      frame.imageStride = cropSize.width();
      frame.image = new Buffer[] { croppedYuv420.createBuffer() };
      frame.opaque = croppedYuv420;
      cropSize.close();

      ExceptionTools.handle(() -> recorder.record(frame, avutil.AV_PIX_FMT_YUV420P), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      frame.close();
   }

   public void close()
   {
      ExceptionTools.handle(recorder::stop, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
}
