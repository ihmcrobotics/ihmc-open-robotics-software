package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.swresample.SwrContext;
import org.bytedeco.ffmpeg.swscale.SwsContext;

/***
 * This class basically copies the class found in the muxing.c example in the FFMPEG docs.
 * It serves to provide a consolidated interface for interaction with audio/video streams
 */
public class FFMPEGOutputStream //TODO modify to match Java conventions
{
   //Variables
   private AVStream stream;
   private AVCodecContext encoder;

   private long nextPts;
   private int samplesCount;

   private AVFrame frame;
   private AVFrame tempFrame;

   private AVPacket tempPacket;

   private float t, tincr, tincr2;

   private SwsContext swsContext;

   //Getters and setters
   public AVStream getStream()
   {
      return stream;
   }

   public void setStream(AVStream stream)
   {
      this.stream = stream;
   }

   public AVCodecContext getEncoder()
   {
      return encoder;
   }

   public void setEncoder(AVCodecContext encoder)
   {
      this.encoder = encoder;
   }

   public long getNextPts()
   {
      return nextPts;
   }

   public void setNextPts(long nextPts)
   {
      this.nextPts = nextPts;
   }

   public int getSamplesCount()
   {
      return samplesCount;
   }

   public void setSamplesCount(int samplesCount)
   {
      this.samplesCount = samplesCount;
   }

   public AVFrame getFrame()
   {
      return frame;
   }

   public void setFrame(AVFrame frame)
   {
      this.frame = frame;
   }

   public AVFrame getTempFrame()
   {
      return tempFrame;
   }

   public void setTempFrame(AVFrame tempFrame)
   {
      this.tempFrame = tempFrame;
   }

   public AVPacket getTempPacket()
   {
      return tempPacket;
   }

   public void setTempPacket(AVPacket tempPacket)
   {
      this.tempPacket = tempPacket;
   }

   public float getT()
   {
      return t;
   }

   public void setT(float t)
   {
      this.t = t;
   }

   public float getTincr()
   {
      return tincr;
   }

   public void setTincr(float tincr)
   {
      this.tincr = tincr;
   }

   public float getTincr2()
   {
      return tincr2;
   }

   public void setTincr2(float tincr2)
   {
      this.tincr2 = tincr2;
   }

   public SwsContext getSwsContext()
   {
      return swsContext;
   }

   public void setSwsContext(SwsContext swsContext)
   {
      this.swsContext = swsContext;
   }

   public SwrContext getSwrContext()
   {
      return swrContext;
   }

   public void setSwrContext(SwrContext swrContext)
   {
      this.swrContext = swrContext;
   }

   private SwrContext swrContext;
}
