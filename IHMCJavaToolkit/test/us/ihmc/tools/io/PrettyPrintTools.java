package us.ihmc.tools.io;

import java.util.Map;

public class PrettyPrintTools
{
   public static <K,V> String prettyPrintMap(String mapName, Map<K, V> map)
   {
      String prettyMapString = "[" + map.getClass().getSimpleName() +": " + mapName + "]\n";      
      
      for (K key : map.keySet())
      {
         prettyMapString += key.toString() + ": " + map.get(key).toString() + "\n";
      }
      
      return prettyMapString;
   }
}
