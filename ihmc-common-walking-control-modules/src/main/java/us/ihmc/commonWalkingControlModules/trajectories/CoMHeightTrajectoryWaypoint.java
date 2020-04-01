package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class CoMHeightTrajectoryWaypoint
{
   private final FramePoint3D waypoint = new FramePoint3D();
   private final FramePoint3D minWaypoint = new FramePoint3D();
   private final FramePoint3D maxWaypoint = new FramePoint3D();

   public void setToZero(ReferenceFrame referenceFrame)
   {
      waypoint.setToZero(referenceFrame);
      minWaypoint.setToZero(referenceFrame);
      maxWaypoint.setToZero(referenceFrame);
   }

   public void setX(double x)
   {
      waypoint.setX(x);
      minWaypoint.setX(x);
      maxWaypoint.setX(x);
   }

   public void setY(double y)
   {
      waypoint.setY(y);
      minWaypoint.setY(y);
      maxWaypoint.setY(y);
   }

   public void setXY(double x, double y)
   {
      setX(x);
      setY(y);
   }

   public void setHeight(double height)
   {
      waypoint.setZ(height);
   }

   public void setMinMax(double zMin, double zMax)
   {
      minWaypoint.setZ(zMin);
      maxWaypoint.setZ(zMax);
   }

   public double getX()
   {
      return waypoint.getX();
   }

   public double getHeight()
   {
      return waypoint.getZ();
   }

   public double getMinHeight()
   {
      return minWaypoint.getZ();
   }

   public double getMaxHeight()
   {
      return maxWaypoint.getZ();
   }

   public FramePoint3DReadOnly getWaypoint()
   {
      return waypoint;
   }

   public FramePoint3DReadOnly getMinWaypoint()
   {
      return minWaypoint;
   }

   public FramePoint3DReadOnly getMaxWaypoint()
   {
      return maxWaypoint;
   }
}
