package us.ihmc.communication.configuration;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.EnumMap;
import java.util.Locale;
import java.util.Properties;

import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.text.WordUtils;
import us.ihmc.tools.io.printing.PrintTools;

public class NetworkParameters
{
   public static final String defaultParameterFile = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "IHMCNetworkParameters.ini";
   private static final String helpText = "Please set all appropriate environment variables or use NetworkParametersCreator to create a properties file and save it in " + defaultParameterFile + ", or pass in -DnetworkParameterFile=[path].";

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
      File file = new File(System.getProperty("networkParameterFile", defaultParameterFile)).getAbsoluteFile();
      PrintTools.info("Looking for network parameters in network parameters file at " + file.getAbsolutePath());

      if (file.exists() && file.isFile())
      {
         try
         {
            Properties properties = new Properties();
            FileInputStream stream = new FileInputStream(file);
            properties.load(stream);
            for (NetworkParameterKeys key : NetworkParameterKeys.values())
            {
               String keyString = key.toString();
               if (properties.containsKey(keyString))
               {
                  parameters.put(key, properties.getProperty(keyString));
               }
            }
            stream.close();
         }
         catch (IOException e)
         {
            System.err.println("Network parameter file " + file.getAbsolutePath() + "exists but cannot be loaded. See stack trace.");
            e.printStackTrace();
         }
      }
      else
      {
         PrintTools.warn("Network parameter file " + file.getAbsolutePath() + " does not exist.");
      }

      PrintTools.info("Looking for network parameters in environment variables");
      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         String keyString = key.toString();
         String envVarString = "IHMC_" + StringUtils.join(StringUtils.splitByCharacterTypeCamelCase(keyString), '_').toUpperCase(Locale.getDefault());
         if(key.isIPAddress())
         {
            envVarString += "_IP";
         }

         if(System.getenv().containsKey(envVarString))
         {
            parameters.put(key, System.getenv(envVarString));
         }
      }

      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         String keyString = key.toString();
         String envVarString = "IHMC_" + StringUtils.join(StringUtils.splitByCharacterTypeCamelCase(keyString), '_').toUpperCase(Locale.getDefault());
         if(key.isIPAddress())
         {
            envVarString += "_IP";
         }
         if(key.isRequired() && !parameters.containsKey(key))
         {
            PrintTools.error("Could not find Network Parameter key " + keyString + " (Env. Variable: " + envVarString + ") . Exiting.\n" + helpText);
            System.exit(-1);
         }
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
            return new URI(getHost(NetworkParameterKeys.rosURI));
         }
         catch (URISyntaxException e)
         {
            throw new RuntimeException("Invalid ROS URI" + getHost(NetworkParameterKeys.rosURI), e);
         }
      }
   }
}
