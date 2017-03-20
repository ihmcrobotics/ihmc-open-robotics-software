package us.ihmc.valkyrie.visualizer;

import java.util.HashMap;

import us.ihmc.commons.PrintTools;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionDisplay;

/**
 * Created by dstephen on 12/9/15.
 */
public class ValkyrieIpToNiceNameRemapper implements LogSessionDisplay.RobotIPToNameRemapHandler
{
   private static final HashMap<String, String> map = new HashMap<String, String>(){{
      put("10.185.0.10", "Valkyrie Unit A");
      put("10.185.0.20", "Valkyrie Unit B");
      put("10.185.0.30", "Valkyrie Unit C");
      put("10.185.0.40", "Valkyrie Unit D");
   }};

   @Override public String getRemap(String ipAddress)
   {
      if(map.containsKey(ipAddress))
      {
         return map.get(ipAddress);
      }
      else
      {
         PrintTools.warn("No mapping for IP Address " + ipAddress);
         return ipAddress;
      }
   }
}
