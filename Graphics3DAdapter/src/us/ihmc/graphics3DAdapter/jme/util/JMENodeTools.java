package us.ihmc.graphics3DAdapter.jme.util;

import us.ihmc.graphics3DAdapter.jme.JMERayCastOpacity;

import com.jme3.scene.Spatial;

public class JMENodeTools
{
   public static boolean isVisualization(Spatial spatial)
   {
      String rayCastOpacity = spatial.getUserData(JMERayCastOpacity.USER_DATA_FIELD);
      
      if (rayCastOpacity != null && rayCastOpacity.equals(JMERayCastOpacity.TRANSPARENT.toString()))
      { 
         return true;
      }
      
      return false;
   }
}
