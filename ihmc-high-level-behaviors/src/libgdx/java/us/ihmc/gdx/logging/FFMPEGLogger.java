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
   private AVDictionary dict;
   private AVFormatContext fmtContext;
   private FFMPEGOutputStream videoStream;

   /***
    * Note - lossless is true by default
    */
   FFMPEGLogger(int width, int height) {
      this(width, height, true);
   }

   /***
    * Note - lossless is true by default
    */
   FFMPEGLogger(int width, int height, String logName) {
      this(width, height, true, "Video");
   }

   FFMPEGLogger(int width, int height, boolean lossless) {
      this(width, height, lossless, "Video");
   }

   FFMPEGLogger(int width, int height, boolean lossless, String logName) {
      fileName = logDirectory + dateFormat.format(new Date()) + "_" + logName + "_Log.webm";

      av_dict_set(dict, "lossless", lossless ? "1" : "0", 0); //TODO this is maybe wrong

      LogTools.info("Building FFMPEG contexts");
      //Output context
      avformat_alloc_output_context2(fmtContext, null, null, fileName);
      if (fmtContext == null || fmtContext.isNull()) {
         LogTools.error("Failed to find output format webm (does this computer support webm?). The logger will not begin.");
         isClosed = true;
         return;
      }

      //Add video stream
      videoStream = new FFMPEGOutputStream();
      AVCodecContext codecContext;

      AVCodec codec = avcodec_find_encoder(fmtContext.video_codec_id());

      videoStream.setTempPacket(av_packet_alloc());
      videoStream.setStream(avformat_new_stream(fmtContext, null));
      videoStream.getStream().id(fmtContext.nb_streams() - 1); //I don't know what this does at all but it's in the example
      codecContext = avcodec_alloc_context3(codec);
      videoStream.setEncoder(codecContext);

      codecContext.codec_id(fmtContext.video_codec_id());
      codecContext.bit_rate(400000); //This is what they've used in all the examples but is arbitrary other than that
      codecContext.width(width);
      codecContext.height(height);

      AVRational rate = new AVRational();
      rate.num(1);
      rate.den(FRAMERATE);
      videoStream.getStream().time_base(rate);
      codecContext.time_base(rate);

      codecContext.gop_size(12); //Some or all of these settings may be unnecessary with lossless
      codecContext.pix_fmt(AV_PIX_FMT_RGBA);

      if ((fmtContext.oformat().flags() & AVFMT_GLOBALHEADER) != 0)
         codecContext.flags(codecContext.flags() | AV_CODEC_FLAG_GLOBAL_HEADER);
   }

   /***
    * The first time a frame is put will take longer than the others because of initialization
    * This method DOES NOT handle disposal of frames!
    */
   public boolean put(BytedecoImage image) {
      if (isClosed)
         return false;

      if (!isInitialized) {
         AVCodecContext codecContext = videoStream.getEncoder();
         AVDictionary opt = new AVDictionary();
         av_dict_copy(opt, dict, 0);

         try
         {
            if (avcodec_open2(codecContext, codecContext.codec(), opt) > 0) //TODO codec may be wrong here
            {
               LogTools.error("Could not open video codec. Logging will not begin.");
               close();
               return false;
            }
         }
         finally
         {
            av_dict_free(opt); //Free dictionary even if we return false up there
         }

         AVFrame pic = av_frame_alloc();
         pic.format(codecContext.pix_fmt());
         pic.width(codecContext.width());
         pic.height(codecContext.height());

         videoStream.setFrame(pic);
         if (av_frame_get_buffer(pic, 0) > 0) {
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

         if (avcodec_parameters_from_context(videoStream.getStream().codecpar(), codecContext) > 0) {
            LogTools.error("Could not copy parameters to muxer. Logging will not begin.");
            close();
            return false;
         }

         av_dump_format(fmtContext, 0, fileName, 1); //this is not freeing the memory - that's avformat_free_context (called during close())

         if (avio_open(fmtContext.pb(), fileName, AVIO_FLAG_WRITE) < 0) {
            LogTools.error("Could not open file for writing. Logging will not begin.");
            close();
            return false;
         }

         avformat_write_header(fmtContext, opt);

         isInitialized = true; //Initialization is now finished. Note that !isInitialized && isClosed is an error state
      }

      //Encode video
      AVFrame frame = getVideoFrame(videoStream);
      avcodec_send_frame(videoStream.getEncoder(), frame);

      //This while loop is weird, but copied from muxing.c
      int ret = 0;
      while (ret >= 0) {
         ret = avcodec_receive_packet(videoStream.getEncoder(), videoStream.getTempPacket());
         if ( ret < 0) {
            LogTools.error("Error encoding frame. Logging will stop.");
            close();
            return false;
         }

         av_packet_rescale_ts(videoStream.getTempPacket(), videoStream.getEncoder().time_base(), videoStream.getStream().time_base());
         videoStream.getTempPacket().stream_index(videoStream.getStream().index());

         ret = av_interleaved_write_frame(fmtContext, videoStream.getTempPacket());
         if (ret < 0) {
            LogTools.error("Error writing output packet. Logging will stop.");
            close();
            return false;
         }
      }

      return ret == AVERROR_EOF();
   }

   private AVFrame getVideoFrame(FFMPEGOutputStream ost) {
      AVCodecContext c = ost.getEncoder();

      AVRational one = new AVRational().num(1).den(1);
      if (av_compare_ts(videoStream.getNextPts(), c.time_base(), 10, one) > 0) //Value 10 comes from STREAM_DURATION in file, probably should be higher but call may be doing other things
         return null;

      if (av_frame_make_writable(ost.getFrame()) < 0) {
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

      videoStream.getFrame().pts(videoStream.getNextPts());

      return videoStream.getFrame();
   }

   public void close() {
      LogTools.info("Closing logger (if you did not expect this to happen, something has gone wrong, and logging will stop.)");

      avcodec_free_context(videoStream.getEncoder());

      av_frame_free(videoStream.getFrame());
      if (videoStream.getTempFrame() != null && !videoStream.getTempFrame().isNull())
         av_frame_free(videoStream.getTempFrame());

      av_packet_free(videoStream.getTempPacket());

      sws_freeContext(videoStream.getSwsContext());
      swr_free(videoStream.getSwrContext());

      isClosed = true;
   }

   public boolean isClosed() {
      return isClosed;
   }
}
