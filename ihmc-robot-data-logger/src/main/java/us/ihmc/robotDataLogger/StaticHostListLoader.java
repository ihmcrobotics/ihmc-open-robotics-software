package us.ihmc.robotDataLogger;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.idl.serializers.extra.YAMLSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;

public class StaticHostListLoader
{
   public static final String location = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "ControllerHosts.yaml";
   
   public static List<HTTPDataServerDescription> load()
   {
      YAMLSerializer<StaticHostList> ser = new YAMLSerializer<StaticHostList>(new StaticHostListPubSubType());
      ser.setAddTypeAsRootNode(false);
      
      File in = new File(location);
      try
      {
         if (in.exists())
         {
            List<HTTPDataServerDescription> list = new ArrayList<>();
            StaticHostList hostList = ser.deserialize(in);
            for(Host host : hostList.getHosts())
            {
               HTTPDataServerDescription description = new HTTPDataServerDescription(host.getHostnameAsString(), host.getPort(), true);
               list.add(description);
            }
            
            return list;
         }
         else
         {
            LogTools.warn("Cannot find " + location + ". Starting with empty list of hosts.");
         }
      }
      catch (IOException e)
      {
         LogTools.warn("Cannot load hosts list: " + e.getMessage());
      }
      
      return Collections.emptyList();
   }
   
   public static void save(List<HTTPDataServerDescription> list) throws IOException
   {
      YAMLSerializer<StaticHostList> ser = new YAMLSerializer<StaticHostList>(new StaticHostListPubSubType());
      ser.setAddTypeAsRootNode(false);
      
      File in = new File(location);
      if(!in.getParentFile().exists())
      {
         in.getParentFile().mkdirs();
      }
      
      StaticHostList staticHostList = new StaticHostList();
      for(HTTPDataServerDescription description : list)
      {
         Host host = staticHostList.getHosts().add();
         host.setHostname(description.getHost());
         host.setPort(description.getPort());
      }
      
      ser.serialize(in, staticHostList);

   }


}
