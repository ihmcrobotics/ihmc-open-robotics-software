package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

import us.ihmc.log.LogTools;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

public class GDXFFMPEGLogger
{
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private String fileName = null;
   private OutputStream file = null;
   private boolean isClosed = false;
   private AVCodec codec;
   private AVCodecContext context;

   /***
    * Note - lossless is true by default
    */
   GDXFFMPEGLogger(int width, int height) {
      this(width, height, true);
   }

   /***
    * Note - lossless is true by default
    */
   GDXFFMPEGLogger(int width, int height, String logName) {
      this(width, height, true, "Video");
   }

   GDXFFMPEGLogger(int width, int height, boolean lossless) {
      this(width, height, lossless, "Video");
   }

   GDXFFMPEGLogger(int width, int height, boolean lossless, String logName) {
      fileName = logDirectory + dateFormat.format(new Date()) + "_" + logName + "_Log.webm";

      //AV context
      LogTools.info("Building FFMPEG AVCodecContext");
      codec = avcodec_find_encoder(AV_CODEC_ID_VP9);
      if (codec == null || codec.isNull()) //codec == null may be unnecessary
      {
         LogTools.error("VP9 Codec not found! Logging will not begin."); //TODO May have to include --enable-libvpx somewhere for ffmpeg
         isClosed = true;
         return;
      }

      context = avcodec_alloc_context3(codec);
      if (lossless)
         context.flags(AVCodecContext.FF_CODEC_PROPERTY_LOSSLESS); //this is probably right
      context.width(width);
      context.height(height);
      context.pix_fmt(AV_PIX_FMT_RGBA); //RGBA8
      //TODO (maybe) something may or may not be missing (this is probably right)
      //Possibly missing: bit_rate, time_base, gop_size, max_b_frames, or maybe more!
   }

   /***
    * This method DOES NOT handle disposal of frames!
    */
   public boolean put(AVFrame frame) { //TODO this method does not work
      if (isClosed)
         return false;

      if (file == null) {
         try
         {
            file = new FileOutputStream(fileName);

            if (avcodec_open2(context, codec, new AVDictionary()) != 0) { //this is probably wrong
               LogTools.error("Something went wrong. Logging will not begin.");
               isClosed = true;
               return false;
            }
         }
         catch(FileNotFoundException ex) {
            LogTools.error("Unable to create logfile " + fileName + "! Logging will not begin.");
         }
      }

      //TODO write AVFrame to video file. This method may need to be private, and the AVFrame might need to be created within the context established in this file.
      //avcodec_encode_video() will be helpful

      //Flush at some point
      try
      {
         file.flush();
      }
      catch(IOException ex) {
         LogTools.warn("Unable to flush logfile stream (non-critical)");
      }
      return false;
   }

   public void close() {
      LogTools.info("Closing logger (if you did not expect this to happen, something has gone wrong, and logging will stop.)");

      try {
         file.flush();
         file.close();
      }
      catch(IOException ex) {
         LogTools.warn("Something went wrong while closing logger - log may not have been saved properly.");
      }

      isClosed = true;
   }

   public boolean isClosed() {
      return isClosed;
   }
}
