package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.FFmpegFrameRecorder;
import org.bytedeco.javacv.Frame;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.variable.YoLong;

import java.nio.file.Path;

import static us.ihmc.zed.global.zed.*;
import static us.ihmc.zed.global.zed.SL_MEM_CPU;

public class LeRobotDatasetEpisode
{
   private final int episodeIndex;
   private final String episodeName;
   private final String taskName;
   private final Path episodesJsonlPath;
   private final Path episodeStatsJsonlPath;
   private final Path dataChunk0Path;
   private final SideDependentList<Path> zedVideoDirs;

   private int length = 0; // TODO
   private final SideDependentList<FFmpegFrameRecorder> ffmpegRecorders = new SideDependentList<>();

   public LeRobotDatasetEpisode(int episodeIndex,
                                String taskName,
                                Path episodesJsonlPath,
                                Path episodeStatsJsonlPath,
                                Path dataChunk0Path,
                                SideDependentList<Path> zedVideoDirs)
   {
      this.episodeIndex = episodeIndex;
      this.taskName = taskName;
      this.episodesJsonlPath = episodesJsonlPath;
      this.episodeStatsJsonlPath = episodeStatsJsonlPath;
      this.dataChunk0Path = dataChunk0Path;
      this.zedVideoDirs = zedVideoDirs;

      episodeName = "episode_%06d".formatted(episodeIndex);
   }

   public void startGeneratingEpisode(SCS2LogSessionWithVideo session)
   {
      YoLong yoTimestamp = session.getLogDataReader().getTimestamp();

      LeRobotDatasetTools.appendLine(episodesJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ArrayNode tasksArray = node.putArray("tasks");
         tasksArray.add(taskName);
         node.put("length", length);
      }));

      //      session.getLogDataReader().
      //      session.get

      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      double sessionDTSeconds = session.getSessionDTSeconds();
      LogTools.info("dt: {}", sessionDTSeconds);

      // TODO: merely start the mp4 export
      //      LeRobotDatasetTools.extractMP4FromZED(zedSVOScrubber);

      ThreadTools.startAsDaemon(() ->
      {
         session.submitBufferIndexRequestAndWait(inPoint);

         ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);
         int imageHeight = zedSVOScrubber.getImageHeight();
         int imageWidth = zedSVOScrubber.getImageWidth();

         for (RobotSide side : RobotSide.values)
         {
            Path mp4Path = zedVideoDirs.get(side).resolve(episodeName + ".mp4");

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

            FFmpegFrameRecorder recorder = new FFmpegFrameRecorder(mp4Path.toString(), imageWidth, imageHeight);

            recorder.setFormat("mp4");
            recorder.setVideoCodec(avcodec.AV_CODEC_ID_AV1);
            recorder.setPixelFormat(avutil.AV_PIX_FMT_YUV420P);
            double fps = 50.0;
            recorder.setFrameRate(fps);

            ExceptionTools.handle(() -> recorder.start(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

            ffmpegRecorders.put(side, recorder);
         }

         int numberOfFrames = outPoint - inPoint;
         for (int i = 0; i < numberOfFrames; i++)
         {
            session.playbackTick();

            if (i % 100 == 0) // TODO: Reduce to FPS - data rate
            {
               long timestamp = yoTimestamp.getLongValue();
               zedSVOScrubber.scrub(timestamp);

               for (RobotSide side : RobotSide.values)
               {
                  Pointer zedColorImageSLMatPointer =
                        side == RobotSide.LEFT ? zedSVOScrubber.getLeftColorImageSlMatPointer() : zedSVOScrubber.getRightColorImageSlMatPointer();
                  Mat bgra8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                                         sl_mat_get_ptr(zedColorImageSLMatPointer, SL_MEM_CPU), sl_mat_get_step_bytes(zedColorImageSLMatPointer, SL_MEM_CPU));

                  // Resize smaller for better transformer training
                  Size size = new Size(853, 480);
                  Mat resized = new Mat(size.height(), size.width(), opencv_core.CV_8UC4);
                  opencv_imgproc.resize(bgra8Mat, resized, size);
                  size.close();

                  // TODO: Crop the sides off to 640x480?

                  FFmpegFrameRecorder recorder = ffmpegRecorders.get(side);

                  Frame frame = new Frame();
                  ExceptionTools.handle(() -> recorder.record(frame), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

                  resized.close();
               }
            }
         }

         for (RobotSide side : RobotSide.values)
         {
            ExceptionTools.handle(() -> ffmpegRecorders.get(side).stop(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }

      }, getClass().getSimpleName());
   }

   public String getEpisodeName()
   {
      return episodeName;
   }

   public Path getEpisodesJsonlPath()
   {
      return episodesJsonlPath;
   }

   public Path getEpisodeStatsJsonlPath()
   {
      return episodeStatsJsonlPath;
   }

   public Path getDataChunk0Path()
   {
      return dataChunk0Path;
   }

   public SideDependentList<Path> getZedVideoDirs()
   {
      return zedVideoDirs;
   }

   public int getLength()
   {
      return length;
   }
}
