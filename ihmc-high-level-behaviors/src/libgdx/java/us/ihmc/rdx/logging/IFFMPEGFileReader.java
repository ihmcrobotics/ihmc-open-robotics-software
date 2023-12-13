package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avutil.AVRational;

import java.nio.ByteBuffer;

public interface IFFMPEGFileReader
{
   long seek(long timestamp);

   long getNextFrame(boolean load);

   void close();

   ByteBuffer getFrameDataBuffer();

   AVRational getTimeBase();

   AVRational getAverageFramerate();

   int getWidth();

   int getHeight();

   /**
    * @return Duration in unit of time base.
    */
   long getDuration();

   /**
    * @return Start time in unit of time base.
    */
   long getStartTime();
}
