package us.ihmc.robotDataLogger;

import java.io.File;
import java.io.IOException;

import gnu.trove.list.array.TByteArrayList;
import us.ihmc.idl.serializers.extra.PropertiesSerializer;

public class LoggerConfigurationLoader
{

   public static final String location = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "IHMCLoggerConfiguration.ini";
   
   private final boolean publicBroadcast;
   private final TByteArrayList cameras = new TByteArrayList();
   
   public LoggerConfigurationLoader() throws IOException
   {

      PropertiesSerializer<LoggerConfiguration> ser = new PropertiesSerializer<>(new LoggerConfigurationPubSubType());

      boolean publicBroadcast = false;
      String cameraString = null;
      
      File in = new File(location);
      if(in.exists())
      {
         LoggerConfiguration config = ser.deserialize(in);
         publicBroadcast = config.getPublicBroadcast();
         cameraString = config.getCamerasToCaptureAsString();
         
      }
      
      
      String publicFromCmd = System.getProperty("ihmc.publicBroadcast");
      String cameraFromCmd = System.getProperty("ihmc.camerasToCapture");
      
      
      if(publicFromCmd != null)
      {
         publicBroadcast = Boolean.parseBoolean(publicFromCmd);
      }
      if(cameraFromCmd != null)
      {
         cameraString = cameraFromCmd;
      }
      
      if(!publicBroadcast)
      {
         System.err.println("Public broadcasting of logger data is OFF. The logger will only connect to your local computer. To enable public broadcasting, add \"publicBroadcast=true\" to " + location + " or pass in -Dihmc.publicBroadcast=true");
      }
      
      this.publicBroadcast = publicBroadcast;
      
      
      
      
      if(cameraString != null && !cameraString.trim().isEmpty())
      {
         String[] split = cameraString.split(",");
         for(int i = 0; i < split.length; i++)
         {
            try
            {
               byte camera = Byte.parseByte(split[i].trim());
               if(camera >= 0 && camera <= 127)
               {
                  cameras.add(camera);
               }
               else
               {
                  throw new NumberFormatException();
               }
            }
            catch(NumberFormatException e)
            {
               throw new IOException("Invalid camera id " + split[i].trim() +",  Please edit " + location + " or pass in a camera list with -Dihmc.camerasToCapture=[cameras, comma seperated]");
            }
         }
      }

   }
   
   

   public boolean getPublicBroadcast()
   {
      return publicBroadcast;
   }



   public TByteArrayList getCameras()
   {
      return cameras;
   }



   public static void main(String[] args) throws IOException
   {
      new LoggerConfigurationLoader();
      
      LoggerConfiguration config = new LoggerConfiguration();
      LoggerConfigurationPubSubType type = new LoggerConfigurationPubSubType();

      PropertiesSerializer<LoggerConfiguration> ser = new PropertiesSerializer<>(type);

      System.out.println(ser.serializeToString(config));
   }
}
