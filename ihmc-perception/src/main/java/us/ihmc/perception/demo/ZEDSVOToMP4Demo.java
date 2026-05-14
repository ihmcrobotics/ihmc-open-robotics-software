package us.ihmc.perception.demo;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.opencv.global.opencv_imgproc;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ffmpeg.FFmpegSoftwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.ffmpeg.FFmpegVideoEncoder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;
import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_NONE;

/**
 * Convert a ZED SVO/SVO2 file into MP4 using IHMC's ZED and FFmpeg bindings.
 *
 * Usage:
 * {@code ZEDSVOToMP4Demo <input.svo2> <output.mp4> [ZED_MODEL] [preferred_encoder]}
 *
 * Example:
 * {@code ZEDSVOToMP4Demo /data/recording.svo2 /tmp/output.mp4 ZED_2I libx264}
 */
public class ZEDSVOToMP4Demo
{
   private static final int OUTPUT_PIXEL_FORMAT = AV_PIX_FMT_YUV420P;
   private static final int INPUT_PIXEL_FORMAT = AV_PIX_FMT_BGR24;
   private static final int MAX_B_FRAMES = 0;

   public static void main(String[] args)
   {
      if (args.length < 2)
      {
         LogTools.info("Usage: ZEDSVOToMP4Demo <input.svo2> <output.mp4> [ZED_MODEL] [preferred_encoder]");
         LogTools.info("Default ZED model: {}", ZEDModelData.ZED_2I.name());
         LogTools.info("Example encoder values: libx264, hevc_nvenc, mpeg4");
         return;
      }

      String inputSVOPath = args[0];
      String outputMP4Path = args[1];
      ZEDModelData zedModel = args.length >= 3 ? ZEDModelData.valueOf(args[2].toUpperCase()) : ZEDModelData.ZED_2I;
      String preferredEncoder = args.length >= 4 ? args[3] : null;

      ZEDSVOPlaybackSensor zed = null;
      FFmpegVideoEncoder encoder = null;
      AVFormatContext outputFormatContext = null;
      AVIOContext outputIOContext = null;
      AVStream outputStream = null;
      AVPacket packetCopy = null;
      RawImage frame = null;
      boolean headerWritten = false;

      try
      {
         zed = new ZEDSVOPlaybackSensor(0, zedModel, SL_DEPTH_MODE_NONE, inputSVOPath);
//         zed.getInitParameters().svo_real_time_mode(false);
         zed.run(true);

         frame = waitForNextColorFrame(zed);
         int imageWidth = frame.getWidth();
         int imageHeight = frame.getHeight();
         int fps = Math.max(15, zed.getFps());
         int bitRate = 10 * imageWidth * imageHeight;
         int gopSize = Math.max(2, fps);

         AVOutputFormat outputFormat = av_guess_format("mp4", outputMP4Path, null);
         FFmpegTools.checkPointer(outputFormat, "Guessing MP4 output format");

         encoder = new FFmpegSoftwareVideoEncoder(outputFormat,
                                                  preferredEncoder,
                                                  bitRate,
                                                  imageWidth,
                                                  imageHeight,
                                                  OUTPUT_PIXEL_FORMAT,
                                                  gopSize,
                                                  MAX_B_FRAMES,
                                                  INPUT_PIXEL_FORMAT);
         encoder.setIntermediateColorConversion(opencv_imgproc.COLOR_BGRA2BGR);
         encoder.initialize();

         outputFormatContext = new AVFormatContext();
         int error = avformat_alloc_output_context2(outputFormatContext, outputFormat, (String) null, outputMP4Path);
         FFmpegTools.checkNegativeError(error, "Allocating output format context");

         outputIOContext = new AVIOContext();
         error = avio_open2(outputIOContext, outputMP4Path, AVIO_FLAG_WRITE, null, null);
         FFmpegTools.checkNegativeError(error, "Opening output file");
         outputFormatContext.pb(outputIOContext);

         outputStream = encoder.newStream(outputFormatContext);

         AVDictionary muxerOptions = new AVDictionary();
         av_dict_set(muxerOptions, "movflags", "+faststart", 0);
         error = avformat_write_header(outputFormatContext, muxerOptions);
         FFmpegTools.checkNegativeError(error, "Writing MP4 header");
         FFmpegTools.checkDictionaryAfterUse(muxerOptions);
         av_dict_free(muxerOptions);
         muxerOptions.close();
         headerWritten = true;

         packetCopy = av_packet_alloc();
         FFmpegTools.checkPointer(packetCopy, "Allocating packet copy");

         long encodedFrameCount = 0;
         int lastSVOPosition = -1;

         int printCount = 0;
         while (true)
         {
            if (printCount++ % 50 == 0)
               LogTools.info("Working...");

            int currentSVOPosition = zed.getCurrentPosition();
            // ZED resets position to the beginning at EOF; use that as stop condition.
            if (lastSVOPosition >= 0 && currentSVOPosition <= lastSVOPosition)
            {
               frame.release();
               break;
            }
            lastSVOPosition = currentSVOPosition;

            encodeAndWriteFrame(encoder, outputFormatContext, outputStream, packetCopy, frame);
            frame.release();
            encodedFrameCount++;

            frame = waitForNextColorFrame(zed);
         }

         flushAndWritePackets(encoder, packetCopy, outputStream, outputFormatContext);
         FFmpegTools.checkNegativeError(av_write_trailer(outputFormatContext), "Writing MP4 trailer");
         headerWritten = false;

         LogTools.info("Finished converting {} to {} ({} frames)", inputSVOPath, outputMP4Path, encodedFrameCount);
      }
      catch (InterruptedException interruptedException)
      {
         Thread.currentThread().interrupt();
         throw new RuntimeException("Interrupted while reading SVO frames", interruptedException);
      }
      finally
      {
         if (frame != null)
            frame.release();

         if (packetCopy != null)
         {
            av_packet_free(packetCopy);
            packetCopy.close();
         }

         if (headerWritten && outputFormatContext != null)
            av_write_trailer(outputFormatContext);

         if (outputIOContext != null)
         {
            avio_closep(outputIOContext);
            outputIOContext.close();
         }

         if (outputStream != null)
            outputStream.close();

         if (outputFormatContext != null)
         {
            avformat_free_context(outputFormatContext);
            outputFormatContext.close();
         }

         if (encoder != null)
            encoder.destroy();

         if (zed != null)
            zed.close();
      }
   }

   private static RawImage waitForNextColorFrame(ZEDSVOPlaybackSensor zed) throws InterruptedException
   {
      RawImage frame = null;
      while (frame == null)
      {
         zed.waitForGrab();
         frame = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      }

      return frame;
   }

   private static void encodeAndWriteFrame(FFmpegVideoEncoder encoder,
                                           AVFormatContext outputFormatContext,
                                           AVStream outputStream,
                                           AVPacket packetCopy,
                                           RawImage frame)
   {
      encoder.setNextFrameAcquisitionTime(frame.getAcquisitionTime().toEpochMilli());
      encoder.setNextFrame(frame.getCpuImageMat());
      encoder.encodeNextFrame(packet -> writePacket(packet, packetCopy, encoder, outputStream, outputFormatContext));
   }

   private static void flushAndWritePackets(FFmpegVideoEncoder encoder,
                                            AVPacket packetCopy,
                                            AVStream outputStream,
                                            AVFormatContext outputFormatContext)
   {
      encoder.flush(packet -> writePacket(packet, packetCopy, encoder, outputStream, outputFormatContext));
   }

   private static void writePacket(AVPacket packet,
                                   AVPacket packetCopy,
                                   FFmpegVideoEncoder encoder,
                                   AVStream outputStream,
                                   AVFormatContext outputFormatContext)
   {
      av_packet_ref(packetCopy, packet);
      av_packet_rescale_ts(packetCopy, encoder.getTimeBase(), outputStream.time_base());
      packetCopy.stream_index(outputStream.index());

      int error = av_interleaved_write_frame(outputFormatContext, packetCopy);
      FFmpegTools.checkNegativeError(error, "Writing encoded packet");

      av_packet_unref(packetCopy);
   }
}
