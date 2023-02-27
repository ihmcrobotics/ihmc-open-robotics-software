package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FixedFrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoLong;

public class YoFrameSE3Waypoint implements FrameSE3WaypointBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;
   private final YoFrameEuclideanWaypoint euclideanWaypoint;
   private final YoFrameSO3Waypoint so3Waypoint;

   public YoFrameSE3Waypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this(namePrefix,
           nameSuffix,
           new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry),
           new FrameIndexMap.FrameIndexHashMap(),
           registry);
   }

   public YoFrameSE3Waypoint(String namePrefix, String nameSuffix, YoLong frameIndex, FrameIndexMap frameIndexMap, YoRegistry registry)
   {
      this.frameId = frameIndex;
      this.frameIndexMap = frameIndexMap;

      euclideanWaypoint = new YoFrameEuclideanWaypoint(namePrefix, nameSuffix, frameId, frameIndexMap, registry);
      so3Waypoint = new YoFrameSO3Waypoint(namePrefix, nameSuffix, frameId, frameIndexMap, registry);
   }

   @Override
   public FixedFrameEuclideanWaypointBasics getEuclideanWaypoint()
   {
      return euclideanWaypoint;
   }

   @Override
   public FixedFrameSO3WaypointBasics getSO3Waypoint()
   {
      return so3Waypoint;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return YoMutableFrameObject.super.getReferenceFrame();
   }

   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getEuclideanWaypoint(), getSO3Waypoint());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameSE3WaypointReadOnly)
         return equals((FrameSE3WaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
