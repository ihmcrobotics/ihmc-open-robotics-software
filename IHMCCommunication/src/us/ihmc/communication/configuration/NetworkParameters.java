package us.ihmc.communication.configuration;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.EnumMap;
import java.util.Properties;

import us.ihmc.tools.io.printing.PrintTools;

public class NetworkParameters
{
   private static final String helpText = "Please use NetworkParametersCreator to create one and place it in the working directory, or pass in -DnetworkParameterFile=[path].";
   public static final String defaultParameterFile = "IHMCNetworkParameters.ini";

   private static NetworkParameters instance = null;

   private static synchronized NetworkParameters getInstance()
   {
      if (instance == null)
      {
         instance = new NetworkParameters();
      }
      return instance;
   }

   private final EnumMap<NetworkParameterKeys, String> parameters = new EnumMap<>(NetworkParameterKeys.class);

   private NetworkParameters()
   {
      File file = new File(System.getProperty("networkParameterFile", defaultParameterFile));
      PrintTools.info("Loading network parameters from " + file.getAbsolutePath());
      if (file.exists() && file.isFile())
      {
         try
         {
            Properties properties = new Properties();
            FileInputStream stream = new FileInputStream(file);
            properties.load(stream);
            for (NetworkParameterKeys key : NetworkParameterKeys.values())
            {
               if (properties.containsKey(key.toString()))
               {
                  parameters.put(key, properties.getProperty(key.toString()));
               }
            }
            stream.close();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Network parameter file " + file.getAbsolutePath() + " cannot be loaded. " + helpText, e);
         }
      }
      else
      {
         throw new RuntimeException("Network parameter file " + file.getAbsolutePath() + " does not exist. " + helpText);
      }
   }

   public static String getHost(NetworkParameterKeys key)
   {
      return getInstance().parameters.get(key);
   }

   public static URI getROSURI()
   {
      if(getHost(NetworkParameterKeys.rosURI) == null)
      {
         return null;
      }
      else
      {
         try
         {
            return new URI("http://" + getHost(NetworkParameterKeys.rosURI));
         }
         catch (URISyntaxException e)
         {
            throw new RuntimeException("Invalid ROS URI" + getHost(NetworkParameterKeys.rosURI), e);
         }
      }
   }
}
