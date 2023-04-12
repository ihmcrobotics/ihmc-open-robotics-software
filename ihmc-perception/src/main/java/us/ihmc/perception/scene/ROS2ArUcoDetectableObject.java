package us.ihmc.perception.scene;

import us.ihmc.perception.arUco.ArUcoMarkerInfo;

/**
 * A scene object detectable via an ArUco marker.
 * Loads a stored property set for the marker information.
 */
public class ROS2ArUcoDetectableObject extends ROS2DetectableSceneObject
{
   private final ArUcoMarkerInfo arUcoMarkerInfo;

   public ROS2ArUcoDetectableObject(String name)
   {
      super(name);

      arUcoMarkerInfo = new ArUcoMarkerInfo(name);
   }

   public ArUcoMarkerInfo getArUcoMarkerInfo()
   {
      return arUcoMarkerInfo;
   }
}
