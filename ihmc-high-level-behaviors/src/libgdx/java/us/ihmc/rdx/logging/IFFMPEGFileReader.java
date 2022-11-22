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

   AVRational getFramerate();

   int getWidth();

   int getHeight();

   long getDuration();

   long getStartTime();
}
