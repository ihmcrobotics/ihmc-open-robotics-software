package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class CentroidalMotionSupportPolygon
{
   private double startTime;
   private double endTime;
   private final FrameConvexPolygon2d supportPolygon;

   public CentroidalMotionSupportPolygon()
   {
      this(ReferenceFrame.getWorldFrame());
   }
   
   public CentroidalMotionSupportPolygon(ReferenceFrame referenceFrame)
   {
      this.supportPolygon = new FrameConvexPolygon2d(referenceFrame);
   }
   
   public void setStartTime(double startTimeToSet)
   {
      this.startTime = startTimeToSet;
   }
   
   public void setEndTime(double endTimeToSet)
   {
      this.endTime = endTimeToSet;
   }
   
   public double getStartTime()
   {
      return startTime;
   }
   
   public double getEndTime()
   {
      return endTime;
   }
   
   public void setSupportPolygon(FrameConvexPolygon2d supportPolygonToStore)
   {
      this.supportPolygon.setIncludingFrame(supportPolygonToStore);
   }
   
   public void getSupportPolygon(FrameConvexPolygon2d supportPolygonToSet)
   {
      supportPolygonToSet.setIncludingFrame(this.supportPolygon);
   }

   public boolean containsNaN()
   {
      return Double.isNaN(startTime) || Double.isNaN(endTime) || supportPolygon.containsNaN();
   }

   public void set(CentroidalMotionSupportPolygon other)
   {
      this.startTime = other.startTime;
      this.endTime = other.endTime;
      this.supportPolygon.setIncludingFrame(other.supportPolygon);
   }

   public void getSupportPolygon(ConvexPolygon2D tempConvexPolygon)
   {
      tempConvexPolygon.set(this.supportPolygon.getConvexPolygon2d());
   }

   public void reset()
   {
      startTime = Double.NaN;
      endTime = Double.NaN;
      supportPolygon.clear();
   }
}

