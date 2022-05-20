package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;
import static org.bytedeco.ffmpeg.global.swresample.*;

import org.bytedeco.ffmpeg.avutil.AVRational;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Doxygen:
 * https://ffmpeg.org/doxygen/trunk/index.html
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

      av_dict_set(avDictionary, "lossless", lossless ? "1" : "0", 0); //TODO this is maybe wrong

      LogTools.info("Building FFMPEG contexts");
      //Output context
      avformat_alloc_output_context2(avFormatContext, null, null, fileName);
      if (avFormatContext == null || avFormatContext.isNull())
      {
         LogTools.error("Failed to find output format webm (does this computer support webm?). The logger will not begin.");
         isClosed = true;
         return;
      }

      //Add video stream
      videoOutputStream = new FFMPEGOutputStream();
      AVCodecContext avCodecContext;

      AVCodec codec = avcodec_find_encoder(avFormatContext.video_codec_id());

      videoOutputStream.setTempPacket(av_packet_alloc());
      videoOutputStream.setStream(avformat_new_stream(avFormatContext, null));
      videoOutputStream.getStream().id(avFormatContext.nb_streams() - 1); //I don't know what this does at all but it's in the example
      avCodecContext = avcodec_alloc_context3(codec);
      videoOutputStream.setEncoder(avCodecContext);

      avCodecContext.codec_id(avFormatContext.video_codec_id());
      avCodecContext.bit_rate(400000); //This is what they've used in all the examples but is arbitrary other than that
      avCodecContext.width(width);
      avCodecContext.height(height);

      AVRational framePeriod = new AVRational();
      framePeriod.num(1);
      framePeriod.den(FRAMERATE);
      videoOutputStream.getStream().time_base(framePeriod);
      avCodecContext.time_base(framePeriod);

      avCodecContext.gop_size(12); //Some or all of these settings may be unnecessary with lossless
      avCodecContext.pix_fmt(AV_PIX_FMT_RGBA);

      if ((avFormatContext.oformat().flags() & AVFMT_GLOBALHEADER) != 0)
         avCodecContext.flags(avCodecContext.flags() | AV_CODEC_FLAG_GLOBAL_HEADER);
   }

   /***
    * The first time a frame is put will take longer than the others because of initialization
    * This method DOES NOT handle disposal of frames!
    */
   public boolean put(BytedecoImage image)
   {
      if (isClosed)
         return false;

      if (!isInitialized)
      {
         AVCodecContext avCodecContext = videoOutputStream.getEncoder();
         AVDictionary optAVDictionary = new AVDictionary();
         av_dict_copy(optAVDictionary, avDictionary, 0);

         try
         {
            if (avcodec_open2(avCodecContext, avCodecContext.codec(), optAVDictionary) > 0) //TODO codec may be wrong here
            {
               LogTools.error("Could not open video codec. Logging will not begin.");
               close();
               return false;
            }
         }
         finally
         {
            av_dict_free(optAVDictionary); //Free dictionary even if we return false up there
         }

         AVFrame pic = av_frame_alloc();
         pic.format(avCodecContext.pix_fmt());
         pic.width(avCodecContext.width());
         pic.height(avCodecContext.height());

         videoOutputStream.setFrame(pic);
         if (av_frame_get_buffer(pic, 0) > 0)
         {
            LogTools.error("Could not get framebuffer. Logging will not begin.");
            close();
            return false;
         }

         //TODO may be missing frame allocation or something to do with dummy image. \/\/\/

         //TODO review comment from muxing.c
         //It is possible that an additional conversion to YUV420P is happening. I don't know if this is just for MPEG, or a limitation of FFMPEG as a whole:
         /*
          * If the output format is not YUV420P, then a temporary YUV420P
          * picture is needed too. It is then converted to the required
          * output format.
          */
         //For now I'm just not gonna do it (which may cause errors)
         //This is what videoStream.tempFrame is

         if (avcodec_parameters_from_context(videoOutputStream.getStream().codecpar(), avCodecContext) > 0)
         {
            LogTools.error("Could not copy parameters to muxer. Logging will not begin.");
            close();
            return false;
         }

         av_dump_format(avFormatContext, 0, fileName, 1); //this is not freeing the memory - that's avformat_free_context (called during close())

         if (avio_open(avFormatContext.pb(), fileName, AVIO_FLAG_WRITE) < 0)
         {
            LogTools.error("Could not open file for writing. Logging will not begin.");
            close();
            return false;
         }

         avformat_write_header(avFormatContext, optAVDictionary);

         isInitialized = true; //Initialization is now finished. Note that !isInitialized && isClosed is an error state
      }

      //Encode video
      AVFrame frame = getVideoFrame(videoOutputStream);
      avcodec_send_frame(videoOutputStream.getEncoder(), frame);

      //This while loop is weird, but copied from muxing.c
      int ret = 0;
      while (ret >= 0)
      {
         ret = avcodec_receive_packet(videoOutputStream.getEncoder(), videoOutputStream.getTempPacket());
         if (ret < 0)
         {
            LogTools.error("Error encoding frame. Logging will stop.");
            close();
            return false;
         }

         av_packet_rescale_ts(videoOutputStream.getTempPacket(), videoOutputStream.getEncoder().time_base(), videoOutputStream.getStream().time_base());
         videoOutputStream.getTempPacket().stream_index(videoOutputStream.getStream().index());

         ret = av_interleaved_write_frame(avFormatContext, videoOutputStream.getTempPacket());
         if (ret < 0)
         {
            LogTools.error("Error writing output packet. Logging will stop.");
            close();
            return false;
         }
      }

      return ret == AVERROR_EOF();
   }

   private AVFrame getVideoFrame(FFMPEGOutputStream ost)
   {
      AVCodecContext c = ost.getEncoder();

      AVRational one = new AVRational().num(1).den(1);
      if (av_compare_ts(videoOutputStream.getNextPts(), c.time_base(), 10, one)
          > 0) //Value 10 comes from STREAM_DURATION in file, probably should be higher but call may be doing other things
         return null;

      if (av_frame_make_writable(ost.getFrame()) < 0)
      {
         LogTools.error("Could not make frame writable. Logging will stop.");
         close();
         return null;
      }

      //TODO more YUV420P stuff. This may or may not be necessary. It looks like they're converging from the stored YUV image into the codec pixel format, so conversion from whatever we use (RGBA8?) to the codec format is probably necessary here.
      //Note the following C code:
      //      if (c->pix_fmt != AV_PIX_FMT_YUV420P) {
      //         /* as we only generate a YUV420P picture, we must convert it
      //          * to the codec pixel format if needed */
      //         if (!ost->sws_ctx) {
      //            ost->sws_ctx = sws_getContext(c->width, c->height,
      //                                          AV_PIX_FMT_YUV420P,
      //                                          c->width, c->height,
      //                                          c->pix_fmt,
      //                                          SCALE_FLAGS, NULL, NULL, NULL);
      //            if (!ost->sws_ctx) {
      //               fprintf(stderr,
      //                       "Could not initialize the conversion context\n");
      //               exit(1);
      //            }
      //         }
      //         fill_yuv_image(ost->tmp_frame, ost->next_pts, c->width, c->height);
      //         sws_scale(ost->sws_ctx, (const uint8_t * const *) ost->tmp_frame->data,
      //               ost->tmp_frame->linesize, 0, c->height, ost->frame->data,
      //               ost->frame->linesize);
      //      } else {
      //         fill_yuv_image(ost->frame, ost->next_pts, c->width, c->height);
      //      }

      videoOutputStream.getFrame().pts(videoOutputStream.getNextPts());

      return videoOutputStream.getFrame();
   }

   public void close()
   {
      LogTools.info("Closing logger (if you did not expect this to happen, something has gone wrong, and logging will stop.)");

      avcodec_free_context(videoOutputStream.getEncoder());

      av_frame_free(videoOutputStream.getFrame());
      if (videoOutputStream.getTempFrame() != null && !videoOutputStream.getTempFrame().isNull())
         av_frame_free(videoOutputStream.getTempFrame());

      av_packet_free(videoOutputStream.getTempPacket());

      sws_freeContext(videoOutputStream.getSwsContext());
      swr_free(videoOutputStream.getSwrContext());

      isClosed = true;
   }

   public boolean isClosed()
   {
      return isClosed;
   }
}
