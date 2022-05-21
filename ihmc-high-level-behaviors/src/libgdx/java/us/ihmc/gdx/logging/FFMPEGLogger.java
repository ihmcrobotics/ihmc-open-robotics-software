package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.global.swresample;
import org.bytedeco.ffmpeg.global.swscale;
import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Doxygen:
 * https://ffmpeg.org/doxygen/trunk/index.html
 * Wiki:
 * https://trac.ffmpeg.org/
 */
public class FFMPEGLogger
{
   //Constants
   private static final int FRAMERATE = 30;
   //End constants
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private String fileName = null;
   private boolean isInitialized = false;
   private boolean isClosed = false;
   private AVDictionary avDictionary;
   private AVFormatContext avFormatContext;
   private FFMPEGOutputStream videoOutputStream;

   /***
    * Note - lossless is true by default
    */
   FFMPEGLogger(int width, int height)
   {
      this(width, height, true);
   }

   /***
    * Note - lossless is true by default
    */
   FFMPEGLogger(int width, int height, String logName)
   {
      this(width, height, true, "Video");
   }

   FFMPEGLogger(int width, int height, boolean lossless)
   {
      this(width, height, lossless, "Video");
   }

   FFMPEGLogger(int width, int height, boolean lossless, String logName)
   {
      fileName = logDirectory + dateFormat.format(new Date()) + "_" + logName + "_Log.webm";

      int avDictFlags = 0;
      avDictionary = new AVDictionary();
      avutil.av_dict_set(avDictionary, "lossless", lossless ? "1" : "0", avDictFlags); // TODO this is maybe wrong

      LogTools.info("Building FFMPEG contexts");

      // Output context
      avFormatContext = new AVFormatContext();
      int returnCode = avformat.avformat_alloc_output_context2(avFormatContext, null, "webm", fileName);
      if (returnCode != 0 || avFormatContext.isNull())
      {
         LogTools.error("{}: Failed to find output format webm (does this computer support webm?). The logger will not begin.",
                        FFMPEGTools.getErrorCodeString(returnCode));
         isClosed = true;
         return;
      }

      AVOutputFormat outputFormat = avFormatContext.oformat();

      LogTools.debug("Codec ID: " + outputFormat.video_codec());

      // Add video stream
      videoOutputStream = new FFMPEGOutputStream();
      AVCodecContext avCodecContext;

      AVCodec codec = avcodec.avcodec_find_encoder(outputFormat.video_codec());
      if (codec == null)
      {
         LogTools.error("Webm codec is null (does this computer support webm?). The logger will not begin.");
         isClosed = true;
         return;
      }

      videoOutputStream.setTempPacket(avcodec.av_packet_alloc());
      videoOutputStream.setStream(avformat.avformat_new_stream(avFormatContext, null));
      videoOutputStream.getStream().id(avFormatContext.nb_streams() - 1); // I don't know what this does at all but it's in the example
      avCodecContext = avcodec.avcodec_alloc_context3(codec);
      videoOutputStream.setEncoder(avCodecContext);

      avCodecContext.codec_id(avFormatContext.video_codec_id());
      avCodecContext.bit_rate(400000); // This is what they've used in all the examples but is arbitrary other than that
      avCodecContext.width(width);
      avCodecContext.height(height);

      AVRational framePeriod = new AVRational();
      framePeriod.num(1);
      framePeriod.den(FRAMERATE);
      videoOutputStream.getStream().time_base(framePeriod);
      avCodecContext.time_base(framePeriod);

      avCodecContext.gop_size(12); // Some or all of these settings may be unnecessary with lossless
      avCodecContext.pix_fmt(avutil.AV_PIX_FMT_YUV420P);

      if ((outputFormat.flags() & avformat.AVFMT_GLOBALHEADER) != 0)
         avCodecContext.flags(avCodecContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);
   }

   /***
    * The first time a frame is put will take longer than the others because of initialization
    */
   public boolean put(BytedecoImage image)
   {
      if (isClosed)
         return false;

      if (!isInitialized)
      {
         AVCodecContext avCodecContext = videoOutputStream.getEncoder();
         AVDictionary optAVDictionary = new AVDictionary();
         avutil.av_dict_copy(optAVDictionary, avDictionary, 0);

         try
         {
            if (avcodec.avcodec_open2(avCodecContext, avCodecContext.codec(), optAVDictionary) > 0) //TODO codec may be wrong here
            {
               LogTools.error("Could not open video codec. Logging will not begin.");
               close();
               return false;
            }
         }
         finally
         {
            avutil.av_dict_free(optAVDictionary); //Free dictionary even if we return false up there
         }

         AVFrame pic = avutil.av_frame_alloc();
         pic.format(avCodecContext.pix_fmt());
         pic.width(avCodecContext.width());
         pic.height(avCodecContext.height());

         videoOutputStream.setFrame(pic);

         AVFrame tempPic = avutil.av_frame_alloc();
         tempPic.format(avutil.AV_PIX_FMT_YUV420P);
         tempPic.width(avCodecContext.width());
         tempPic.height(avCodecContext.height());

         videoOutputStream.setTempFrame(tempPic);

         if (avutil.av_frame_get_buffer(pic, 0) > 0 || avutil.av_frame_get_buffer(tempPic, 0) > 0)
         {
            LogTools.error("Could not get framebuffer. Logging will not begin.");
            close();
            return false;
         }

         if (avcodec.avcodec_parameters_from_context(videoOutputStream.getStream().codecpar(), avCodecContext) > 0)
         {
            LogTools.error("Could not copy parameters to muxer. Logging will not begin.");
            close();
            return false;
         }

         avformat.av_dump_format(avFormatContext, 0, fileName, 1); //this is not freeing the memory - that's avformat_free_context (called during close())

         try
         {
            new File(fileName).getParentFile().mkdirs();
         }
         catch (Exception ignored) {}

         AVIOContext pb = new AVIOContext();
         int ret = 0;
         if ((ret = avformat.avio_open(pb, fileName, avformat.AVIO_FLAG_WRITE)) < 0)
         {
            LogTools.error("{}: Could not open file for writing. Logging will not begin.", FFMPEGTools.getErrorCodeString(ret));
            close();
            return false;
         }
         avFormatContext.pb(pb);

         if ((ret = avformat.avformat_write_header(avFormatContext, optAVDictionary)) < 0) {
            LogTools.error("{}: Could not write to file. Logging will not begin.", FFMPEGTools.getErrorCodeString(ret));
            close();
            return false;
         }

         isInitialized = true; //Initialization is now finished. Note that !isInitialized && isClosed is an error state
      }

      //Encode video
      AVFrame frame = getVideoFrame(videoOutputStream, image);
      avcodec.avcodec_send_frame(videoOutputStream.getEncoder(), frame);

      //This while loop is weird, but copied from muxing.c
      int ret = 0;
      while (ret >= 0)
      {
         ret = avcodec.avcodec_receive_packet(videoOutputStream.getEncoder(), videoOutputStream.getTempPacket());
         if (ret < 0)
         {
            if (ret == -11) {
               LogTools.warn("{}: This frame will not be logged", FFMPEGTools.getErrorCodeString(ret));
               return false;
            }
            else {
               LogTools.error("{}: Error encoding frame. Logging will stop.", FFMPEGTools.getErrorCodeString(ret));
               close();
               return false;
            }
         }

         avcodec.av_packet_rescale_ts(videoOutputStream.getTempPacket(), videoOutputStream.getEncoder().time_base(), videoOutputStream.getStream().time_base());
         videoOutputStream.getTempPacket().stream_index(videoOutputStream.getStream().index());

         ret = avformat.av_interleaved_write_frame(avFormatContext, videoOutputStream.getTempPacket());
         if (ret < 0)
         {
            LogTools.error("Error writing output packet. Logging will stop.");
            close();
            return false;
         }
      }

      avformat.avio_flush(avFormatContext.pb());

      return ret == avutil.AVERROR_EOF();
   }

   //This method currently only contains random values to see if it's actually printing. TODO convert image into YUVwhatever and print
   private void fillImage(AVFrame pict, BytedecoImage image, int width, int height) {
      int x, y, i;

      for (y = 0; y < height; y++) {
         for (x = 0; x < width; x++) {
            pict.data().get(0).getPointer(y * pict.linesize().get(0) + x).fill(48);
         }
      }

      for (y = 0; y < height / 2; y++) {
         for (x = 0; x < width / 2; x++) {
            pict.data().get(1).getPointer(y * pict.linesize().get(1) + x).fill(52);
            pict.data().get(2).getPointer(y * pict.linesize().get(2) + x).fill(-12);
         }
      }
   }

   private AVFrame getVideoFrame(FFMPEGOutputStream ost, BytedecoImage image)
   {
      AVCodecContext c = ost.getEncoder();

      AVRational one = new AVRational().num(1).den(1);
      if (avutil.av_compare_ts(videoOutputStream.getNextPts(), c.time_base(), 10, one)
          > 0) //Value 10 comes from STREAM_DURATION in file, probably should be higher but call may be doing other things
         return null;

      if (avutil.av_frame_make_writable(ost.getFrame()) < 0)
      {
         LogTools.error("Could not make frame writable. Logging will stop.");
         close();
         return null;
      }

      if (c.pix_fmt() != avutil.AV_PIX_FMT_YUV420P)
      {
         if (ost.getSwsContext() == null || ost.getSwsContext().isNull()) {
            ost.setSwsContext(swscale.sws_getContext(c.width(), c.height(), avutil.AV_PIX_FMT_YUV420P, c.width(), c.height(), c.pix_fmt(), swscale.SWS_BICUBIC, null, null, (DoublePointer)null));

            if (ost.getSwsContext() == null || ost.getSwsContext().isNull()) {
               LogTools.error("Error creating SWS Context.");
               return null;
            }
         }

         fillImage(ost.getTempFrame(), image, c.width(), c.height());

         swscale.sws_scale(ost.getSwsContext(), ost.getTempFrame().data(), ost.getTempFrame().linesize(), 0, c.height(), ost.getFrame().data(), ost.getFrame().linesize());
      }
      else
      {
         fillImage(ost.getFrame(), image, c.width(), c.height());
      }

      videoOutputStream.getFrame().pts(videoOutputStream.getNextPts());
      videoOutputStream.setNextPts(videoOutputStream.getNextPts() + 1);

      return videoOutputStream.getFrame();
   }

   public void close()
   {
      LogTools.info("Closing logger (if you did not expect this to happen, something has gone wrong, and logging will stop.)");
      isClosed = true;

      avformat.avio_close(avFormatContext.pb());

      if (videoOutputStream.getEncoder() != null && !videoOutputStream.getEncoder().isNull())
         avcodec.avcodec_free_context(videoOutputStream.getEncoder());

      if (videoOutputStream.getFrame() != null && !videoOutputStream.getFrame().isNull())
         avutil.av_frame_free(videoOutputStream.getFrame());

      if (videoOutputStream.getTempFrame() != null && !videoOutputStream.getTempFrame().isNull())
         avutil.av_frame_free(videoOutputStream.getTempFrame());

      if (videoOutputStream.getTempPacket() != null && !videoOutputStream.getTempPacket().isNull())
         avcodec.av_packet_free(videoOutputStream.getTempPacket());

      if (videoOutputStream.getSwsContext() != null && !videoOutputStream.getSwsContext().isNull())
         swscale.sws_freeContext(videoOutputStream.getSwsContext());

      if (videoOutputStream.getSwrContext() != null && !videoOutputStream.getSwrContext().isNull())
         swresample.swr_free(videoOutputStream.getSwrContext());
   }

   @Override
   protected void finalize() throws Throwable
   {
      super.finalize();
      close(); //Ensure that file gets written properly
   }

   public boolean isClosed()
   {
      return isClosed;
   }
}
