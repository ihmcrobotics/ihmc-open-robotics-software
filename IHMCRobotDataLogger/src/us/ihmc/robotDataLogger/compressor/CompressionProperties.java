package us.ihmc.robotDataLogger.compressor;

import java.util.Properties;

public class CompressionProperties extends Properties
{

   private static final long serialVersionUID = -4997224794089431370L;

   public void setNumberOfBufferedElements(int elements)
   {
      setProperty("numberOfBufferedElements", String.valueOf(elements));
   }

   public int getNumberOfBufferedElements()
   {
      return Integer.parseInt(getProperty("numberOfBufferedElements"));
   }

   
   public void setCompressedDataFiles(int files)
   {
      setProperty("compressedDataFiles", String.valueOf(files));
   }
   
   public int getCompressedDataFiles()
   {
      return Integer.parseInt(getProperty("compressedDataFiles"));
   }
   
   public void setTimestampChecksum(String sum)
   {
      setProperty("timestampChecksum", sum);
   }

   public String getTimestampChecksum()
   {
      return getProperty("timestampChecksum");
   }
   
   public void setDataChecksum(String sum)
   {
      setProperty("dataChecksum", sum);
   }
   
   public String getDataChecksum()
   {
      return getProperty("dataChecksum");
   }
   
}
