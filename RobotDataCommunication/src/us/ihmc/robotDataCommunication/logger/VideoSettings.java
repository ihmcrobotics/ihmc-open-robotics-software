package us.ihmc.robotDataCommunication.logger;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

public class VideoSettings
{
   /**
    * 
    * Known BlackMagic Mini Recorder modes
    * 
    * mode = 11 -> 1080i59.94Hz
    * mode = 12 -> 1080i60Hz
    * mode = 9 -> 1080p30
    * mode = 14 -> 720p59.94Hz
    * mode = 15 -> 720p60Hz
   */

   private final String description;
   private final int device;
   private final int mode;
   private final boolean interlaced;
   private final VideoIn videoIn;

   public String getDescription()
   {
      return description;
   }

   public int getDevice()
   {
      return device;
   }

   public int getMode()
   {
      return mode;
   }

   public boolean isInterlaced()
   {
      return interlaced;
   }

   private VideoSettings(String description, int device, int mode, boolean interlaced, VideoIn videoIn)
   {
      this.description = description;
      this.device = device;
      this.mode = mode;
      this.interlaced = interlaced;
      this.videoIn = videoIn;
   }

   public VideoIn getVideoIn()
   {
      return videoIn;
   }

   public static List<VideoSettings> loadCameraSettings(InputStream cameraDescription) throws IOException
   {
      ArrayList<VideoSettings> cameraList = new ArrayList<>();
      Yaml yaml = new Yaml();
      
      try
      {
         @SuppressWarnings("unchecked")
         List<Map<String, ?>> root = (List<Map<String, ?>>) yaml.load(cameraDescription);
         
         
         for(Map<String, ?> camera : root)
         {
            String description = (String) camera.get("description");
            int device = (Integer) camera.get("device");
            int mode = (Integer) camera.get("mode");
            boolean interlaced = (Boolean) camera.get("interlaced");
            VideoIn videoIn = VideoIn.valueOf((String) camera.get("videoIn"));
            
            cameraList.add(new VideoSettings(description, device, mode, interlaced, videoIn));
            
         }
      }
      catch(ClassCastException e)
      {
         throw new IOException("Camera description format is invalid", e);
      }
      

      return cameraList;
   }

   @Override
   public String toString()
   {
      return "VideoSettings [description=" + description + ", device=" + device + ", mode=" + mode + ", interlaced=" + interlaced + ", videoIn=" + videoIn
            + "]";
   }
   
   public static void main(String[] args) throws IOException
   {
      InputStream stream = new FileInputStream("cameras.yaml");
      System.out.println(VideoSettings.loadCameraSettings(stream));
      stream.close();
   }
}
