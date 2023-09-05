package us.ihmc.rdx.imgui;

import imgui.extension.imnodes.ImNodes;

public class ImNodesTools
{
   private static boolean INITIALIZED = false;

   public static void destroy()
   {
      if (INITIALIZED)
      {
         ImNodes.destroyContext();
         INITIALIZED = false;
      }
   }

   public static void initialize()
   {
      if (!INITIALIZED)
      {
         INITIALIZED = true;
         ImNodes.createContext();
         if (!Boolean.parseBoolean(System.getProperty("imgui.dark")))
            ImNodes.styleColorsLight();
      }
   }
}
