package us.ihmc.robotDataVisualizer.compressor;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;

public class VideoCompressor
{
   public static final File defaultParameterFile = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "IHMCLogCompressorSettings.ini");

   
   public static final String defaultVideoConverterCmd = "/usr/bin/ffmpeg";
   public static final String defaultVideoCompressOptions = "-i {INPUT} -copyts -c:v libx264 -c:a null -pix_fmt yuv420p -profile:v high -level 4.1 {OUTPUT}";
   public static final String defaultVideoDeCompressOptions = "-i {INPUT} -copyts  -c:v mjpeg -q:v 9.5 {OUTPUT}";
   
   private final Properties logCompressorProperties = new Properties();

   
   public VideoCompressor() throws IOException
   {
      if (defaultParameterFile.exists())
      {
         logCompressorProperties.load(new FileReader(defaultParameterFile));
      }

   }
   
   
   public void setDefaults()
   {
      setVideoCompressorPath(defaultVideoConverterCmd);
      setVideoCompressionOptions(defaultVideoCompressOptions);
      setVideoDecompressionOptions(defaultVideoDeCompressOptions);
   }
   
   public String getVideoCompressorPath()
   {
      return logCompressorProperties.getProperty("videoConverterCmd", VideoCompressor.defaultVideoConverterCmd);   
   }
   
   public String getVideoCompressionOptions()
   {
      return logCompressorProperties.getProperty("videoCompressOptions", VideoCompressor.defaultVideoCompressOptions);
   }
   
   public String getVideoDecompressionOptions()
   {
      return logCompressorProperties.getProperty("videoDecompressOptions", VideoCompressor.defaultVideoDeCompressOptions);
   }
   
   public void setVideoCompressorPath(String path)
   {
      logCompressorProperties.setProperty("videoConverterCmd", path);
   }
   
   public void setVideoCompressionOptions(String options)
   {
      logCompressorProperties.setProperty("videoCompressOptions", options);
   }
   
   public void setVideoDecompressionOptions(String options)
   {
      logCompressorProperties.setProperty("videoDecompressOptions", options);
   }


   public void saveSettings()
   {
      try
      {
         defaultParameterFile.getParentFile().mkdirs();
         FileWriter fileWriter = new FileWriter(defaultParameterFile);
         logCompressorProperties.store(fileWriter, "Saved by LogCompressorUI");
         fileWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
