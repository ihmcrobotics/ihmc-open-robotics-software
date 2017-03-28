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

import us.ihmc.commons.PrintTools;

public class NetworkParameters
{
   public static final String defaultParameterFile = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "IHMCNetworkParameters.ini";
   private static final String helpText =
         "Please set all appropriate environment variables or use NetworkParametersCreator to create a properties file and save it in " + defaultParameterFile
               + ", or pass in -Dus.ihmc.networkParameterFile=[path].";

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
      File file = new File(System.getProperty("us.ihmc.networkParameterFile", defaultParameterFile)).getAbsoluteFile();
      PrintTools.info("Looking for network parameters in network parameters file at " + file.getAbsolutePath());

      if (file.exists() && file.isFile())
      {
         PrintTools.info("Found Network parameters file at " + file.getAbsolutePath());
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
      PrintTools.info("Environment variables will override entries in the network parameters file.");
      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         String keyString = key.toString();
         String envVarString = "IHMC_" + StringUtils.join(StringUtils.splitByCharacterTypeCamelCase(keyString), '_').toUpperCase(Locale.getDefault());
         if (key.isIPAddress())
         {
            envVarString += "_IP";
         }

         if (key == NetworkParameterKeys.rosURI)
         {
            if (System.getenv().containsKey("ROS_MASTER_URI"))
            {
               parameters.put(key, System.getenv("ROS_MASTER_URI"));
            }
         }
         else if (System.getenv().containsKey(envVarString))
         {
            parameters.put(key, System.getenv(envVarString));
         }
      }
   }

   public static String getHost(NetworkParameterKeys key)
   {
      String value = getInstance().parameters.get(key);
      if (value == null)
      {
         String envVarString = "IHMC_" + StringUtils.join(StringUtils.splitByCharacterTypeCamelCase(key.toString()), '_').toUpperCase(Locale.getDefault());
         if (key.isIPAddress())
         {
            envVarString += "_IP";
         }
         if (key != NetworkParameterKeys.rosURI)
         {
            throw new RuntimeException("Could not find " + key.toString() + "! Please check you ini for a " + key.toString()
                  + " and check that the key and value are seperated by a colon. You can use the NetworkParametersCreator to create this file for you. (if using Env. Variables it would be named: "
                  + envVarString + ") . Exiting.\n" + helpText);
         }
         else
         {
            throw new RuntimeException(
                  "Could not establish the ROS Master URI. Check your environment variables for ROS_MASTER_URI or set the IHMC Network key " + key.toString()
                        + " in your ini file. Exiting.\n" + helpText);
         }


//         System.exit(-1);
      }
      return value;
   }

   public static URI getROSURI()
   {
      if (getHost(NetworkParameterKeys.rosURI) == null)
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

   public static boolean hasKey(NetworkParameterKeys key)
   {
      return getInstance().parameters.containsKey(key);
   }
}
