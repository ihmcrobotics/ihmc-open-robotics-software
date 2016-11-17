package us.ihmc.robotDataLogger.compressor;

public class VideoCompessor
{
   public static final String defaultVideoConverterCmd = "/usr/bin/ffmpeg";
   public static final String defaultVideoCompressOptions = "-i {INPUT} -copyts -c:v libx264 -c:a null -pix_fmt yuv420p -profile:v high -level 4.1 {OUTPUT}";
   public static final String defaultVideoDeCompressOptions = "-i {INPUT} -copyts  -c:v mjpeg -q:v 9.5 {OUTPUT}";
}
