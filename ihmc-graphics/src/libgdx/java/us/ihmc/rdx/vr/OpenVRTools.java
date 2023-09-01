package us.ihmc.rdx.vr;

import org.lwjgl.openvr.HmdMatrix34;

public class OpenVRTools
{
   public static boolean containsNaN(HmdMatrix34 hmdMatrix34)
   {
      for (int i = 0; i < 12; i++)
      {
         if (Float.isNaN(hmdMatrix34.m(i)))
         {
            return true;
         }
      }
      return false;
   }
}
