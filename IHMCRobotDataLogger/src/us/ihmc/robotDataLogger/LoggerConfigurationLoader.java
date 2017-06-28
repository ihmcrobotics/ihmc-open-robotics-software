package us.ihmc.robotDataLogger;

import java.io.File;
import java.io.IOException;
import java.net.InetAddress;

import gnu.trove.list.array.TByteArrayList;
import us.ihmc.idl.serializers.extra.PropertiesSerializer;
import us.ihmc.multicastLogDataProtocol.LogUtils;

public class LoggerConfigurationLoader
{

   public static final String location = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "IHMCLoggerConfiguration.ini";
   
   private final InetAddress myNetworkAddress;
   private final TByteArrayList cameras = new TByteArrayList();
   
   public LoggerConfigurationLoader() throws IOException
   {

      PropertiesSerializer<LoggerConfiguration> ser = new PropertiesSerializer<>(new LoggerConfigurationPubSubType());

      String network = null;
      String cameraString = null;
      
      File in = new File(location);
      if(in.exists())
      {
         LoggerConfiguration config = ser.deserialize(in);
         network = config.getLoggerNetworkAsString();
         cameraString = config.getCamerasToCaptureAsString();
         
      }
      
      
      String networkFromCmd = System.getProperty("ihmc.loggerNetwork");
      String cameraFromCmd = System.getProperty("ihmc.camerasToCapture");
      
      
      if(networkFromCmd != null)
      {
         network = networkFromCmd;
      }
      if(cameraFromCmd != null)
      {
         cameraString = cameraFromCmd;
      }
      
      if(network == null)
      {
         throw new IOException("No network to bind to set for the logger. Please edit " + location + " or pass in a correct network with -Dihmc.loggerNetwork=[IP or hostname]");
      }
      
      try
      {
         myNetworkAddress = LogUtils.getMyIP(network);
      }
      catch (IOException e)
      {
         throw new IOException(network + " is not a valid hostname or IP. Please edit " + location + " or pass in a correct network with -Dihmc.loggerNetwork=[IP or hostname]", e);
      }
      
      
      
      
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
   
   

   public InetAddress getMyNetworkAddress()
   {
      return myNetworkAddress;
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
