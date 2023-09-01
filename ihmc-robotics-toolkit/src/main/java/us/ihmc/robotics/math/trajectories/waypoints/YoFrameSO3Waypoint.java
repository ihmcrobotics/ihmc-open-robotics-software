package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoFrameSO3Waypoint implements FrameSO3WaypointBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;
   private final YoMutableFrameQuaternion orientation;
   private final YoMutableFrameVector3D angularVelocity;

   public YoFrameSO3Waypoint(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this(namePrefix,
           nameSuffix,
           new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry),
           new FrameIndexMap.FrameIndexHashMap(),
           registry);
   }

   public YoFrameSO3Waypoint(String namePrefix, String nameSuffix, YoLong frameIndex, FrameIndexMap frameIndexMap, YoRegistry registry)
   {
      this.frameId = frameIndex;
      this.frameIndexMap = frameIndexMap;

      YoDouble qx = new YoDouble(YoGeometryNameTools.createQxName(namePrefix + "Orientation", nameSuffix), registry);
      YoDouble qy = new YoDouble(YoGeometryNameTools.createQyName(namePrefix + "Orientation", nameSuffix), registry);
      YoDouble qz = new YoDouble(YoGeometryNameTools.createQzName(namePrefix + "Orientation", nameSuffix), registry);
      YoDouble qs = new YoDouble(YoGeometryNameTools.createQsName(namePrefix + "Orientation", nameSuffix), registry);
      orientation = new YoMutableFrameQuaternion(qx, qy, qz, qs, frameId, frameIndexMap);
      YoDouble wx = new YoDouble(YoGeometryNameTools.createXName(namePrefix + "AngularVelocity", nameSuffix), registry);
      YoDouble wy = new YoDouble(YoGeometryNameTools.createYName(namePrefix + "AngularVelocity", nameSuffix), registry);
      YoDouble wz = new YoDouble(YoGeometryNameTools.createZName(namePrefix + "AngularVelocity", nameSuffix), registry);
      angularVelocity = new YoMutableFrameVector3D(wx, wy, wz, frameId, frameIndexMap);
   }

   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public FixedFrameVector3DBasics getAngularVelocity()
   {
      return angularVelocity;
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
      return EuclidHashCodeTools.toIntHashCode(getOrientation(), getAngularVelocity());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameSO3WaypointReadOnly)
         return equals((FrameSO3WaypointReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
