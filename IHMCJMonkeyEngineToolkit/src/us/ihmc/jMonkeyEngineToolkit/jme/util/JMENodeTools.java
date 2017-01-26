package us.ihmc.jMonkeyEngineToolkit.jme.util;

import com.jme3.scene.Spatial;

import us.ihmc.jMonkeyEngineToolkit.jme.JMERayCastOpacity;

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
