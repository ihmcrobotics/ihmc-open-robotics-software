package us.ihmc.avatar.logProcessor.leRobot;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.FFmpegFrameRecorder;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;

import java.nio.file.Path;

import static us.ihmc.zed.global.zed.*;
import static us.ihmc.zed.global.zed.SL_MEM_CPU;

public class LeRobotDatasetVideoWriter
{
   private final RobotSide side;
   private final FFmpegFrameRecorder recorder;
   private final OpenCVFrameConverter.ToMat frameConverter = new OpenCVFrameConverter.ToMat();

   // Stats
   private final TIntArrayList redMeans = new TIntArrayList();


   public LeRobotDatasetVideoWriter(RobotSide side, Path mp4Path)
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
      recorder.setFrameRate(LeRobotDataset.ZED_FPS);
      recorder.setVideoBitrate(1352000);

      ExceptionTools.handle(() -> recorder.start(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void writeFrame(ZEDSVOScrubber zedSVOScrubber, long videoTimestampMs)
   {
      int imageHeight = zedSVOScrubber.getImageHeight();
      int imageWidth = zedSVOScrubber.getImageWidth();

      Pointer zedColorImageSLMatPointer =
            side == RobotSide.LEFT ? zedSVOScrubber.getLeftColorImageSlMatPointer() : zedSVOScrubber.getRightColorImageSlMatPointer();
      Mat bgra8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                             sl_mat_get_ptr(zedColorImageSLMatPointer, SL_MEM_CPU),
                             sl_mat_get_step_bytes(zedColorImageSLMatPointer, SL_MEM_CPU));

      // Resize smaller for better transformer training
      Size size = new Size(853, 480);
      Mat resized = new Mat(size, opencv_core.CV_8UC4);
      opencv_imgproc.resize(bgra8Mat, resized, size);
      size.close();

      // Crop the sides of for a minimal 640x480 to make training faster (work?)
      int cropWidth = 640;
      int cropHeight = 480;
      int x = (resized.cols() - cropWidth) / 2;  // Center crop horizontally
      int y = (resized.rows() - cropHeight) / 2; // Center crop vertically
      Rect roi = new Rect(x, y, cropWidth, cropHeight);
      Mat cropped = new Mat(resized, roi);
      resized.close();

      // ZED outputs bgra but frame recorder wants rgba
      opencv_imgproc.cvtColor(cropped, cropped, opencv_imgproc.COLOR_BGR2RGBA);

      // TODO: Calculate stats
      Scalar mean = opencv_core.mean(cropped);

      Frame frame = frameConverter.convert(cropped);
      cropped.close();

      recorder.setTimestamp(videoTimestampMs);
      ExceptionTools.handle(() -> recorder.record(frame), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      frame.close();
   }

   public void close()
   {
      ExceptionTools.handle(recorder::stop, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      frameConverter.close();
   }
}
