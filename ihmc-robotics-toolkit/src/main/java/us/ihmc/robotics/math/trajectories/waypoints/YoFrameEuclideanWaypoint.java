package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoFrameEuclideanWaypoint implements FrameEuclideanWaypointBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;
   private final YoMutableFramePoint3D position;
   private final YoMutableFrameVector3D linearVelocity;

   public YoFrameEuclideanWaypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this(namePrefix,
           nameSuffix,
           new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry),
           new FrameIndexMap.FrameIndexHashMap(),
           registry);
   }

   public YoFrameEuclideanWaypoint(String namePrefix, String nameSuffix, YoLong frameIndex, FrameIndexMap frameIndexMap, YoRegistry registry)
   {
      this.frameId = frameIndex;
      this.frameIndexMap = frameIndexMap;

      YoDouble px = new YoDouble(YoGeometryNameTools.createXName(namePrefix + "Position", nameSuffix), registry);
      YoDouble py = new YoDouble(YoGeometryNameTools.createYName(namePrefix + "Position", nameSuffix), registry);
      YoDouble pz = new YoDouble(YoGeometryNameTools.createZName(namePrefix + "Position", nameSuffix), registry);
      position = new YoMutableFramePoint3D(px, py, pz, frameId, frameIndexMap);
      YoDouble vx = new YoDouble(YoGeometryNameTools.createXName(namePrefix + "LinearVelocity", nameSuffix), registry);
      YoDouble vy = new YoDouble(YoGeometryNameTools.createYName(namePrefix + "LinearVelocity", nameSuffix), registry);
      YoDouble vz = new YoDouble(YoGeometryNameTools.createZName(namePrefix + "LinearVelocity", nameSuffix), registry);
      linearVelocity = new YoMutableFrameVector3D(vx, vy, vz, frameId, frameIndexMap);
   }

   public YoDouble getYoX()
   {
      return position.getYoX();
   }

   public YoDouble getYoY()
   {
      return position.getYoY();
   }

   public YoDouble getYoZ()
   {
      return position.getYoZ();
   }

   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      return position;
   }

   @Override
   public FixedFrameVector3DBasics getLinearVelocity()
   {
      return linearVelocity;
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
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getLinearVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameEuclideanWaypointReadOnly)
         return equals((FrameEuclideanWaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
