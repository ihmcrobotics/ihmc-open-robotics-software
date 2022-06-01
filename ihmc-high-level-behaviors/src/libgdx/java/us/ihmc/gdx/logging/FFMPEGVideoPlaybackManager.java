package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avutil.AVRational;
import us.ihmc.perception.BytedecoImage;

public class FFMPEGVideoPlaybackManager
{
   private final FFMPEGFileReader file;
   private BytedecoImage image;
   private AVRational timeBase;
   private long previousBaseUnitsTimestamp;

   public FFMPEGVideoPlaybackManager(String file) {
      this.file = new FFMPEGFileReader(file);
      this.timeBase = this.file.getTimeBase();
   }

   public void seek(long milliseconds) {
      file.seek(millisToBaseUnits(milliseconds));
   }

   public void play() {

   }

   public void pause() {

   }

   public long getVideoDurationInMillis() {
      return baseUnitsToMillis(file.getDuration());
   }

   public long getCurrentTimestampInMillis() {
      return baseUnitsToMillis(previousBaseUnitsTimestamp);
   }

   private long millisToBaseUnits(long millis) {
      return (long) (millis * FFMPEGTools.rationalToFloatingPoint(timeBase) / 1000);
   }

   private long baseUnitsToMillis(long baseUnits) {
      long millis = 0;
      if (timeBase.num() == 1) //Should basically always be one, but just to be safe
      {
         //This method increases accuracy when the time base is simple (which is often the case)
         millis = baseUnits / timeBase.den() * 1000;
         millis += (baseUnits % timeBase.den()) / FFMPEGTools.rationalToFloatingPoint(timeBase) * 1000;
      }
      else
      {
         millis = (long) (baseUnits / FFMPEGTools.rationalToFloatingPoint(timeBase) * 1000);
      }

      return millis;
   }

   public int getWidth() {
      return file.getWidth();
   }

   public int getHeight() {
      return file.getHeight();
   }

   public void setDestinationImage(BytedecoImage image)
   {
      this.image = image;
   }
}
