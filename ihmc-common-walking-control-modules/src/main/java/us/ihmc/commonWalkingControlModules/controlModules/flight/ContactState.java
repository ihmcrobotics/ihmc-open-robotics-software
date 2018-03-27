package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class ContactState
{
   private ContactStateEnum state;
   private double initialTime;
   private double finalTime;
   private final FrameConvexPolygon2d supportPolygon;

   public ContactState()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ContactState(ReferenceFrame referenceFrame)
   {
      this.supportPolygon = new FrameConvexPolygon2d(referenceFrame);
      reset();
   }

   public void reset()
   {
      state = null;
      initialTime = Double.NaN;
      finalTime = Double.NaN;
      supportPolygon.clear();
   }

   public void setContactStat(ContactStateEnum stateToSet)
   {
      this.state = stateToSet;
   }

   public void setSupportPolygonIncludingFrame(FrameConvexPolygon2d supportPolygonToSet)
   {
      this.supportPolygon.setIncludingFrame(supportPolygonToSet);
   }

   public void setSupportPolygon(FrameConvexPolygon2d supportPolygonToSet)
   {
      this.supportPolygon.set(supportPolygonToSet);
   }

   public void setContactStateInitialTime(double initialTime)
   {
      this.initialTime = initialTime;
   }

   public void setContactStateFinalTime(double finalTime)
   {
      this.finalTime = finalTime;
   }

   public void getSupportPolygon(FrameConvexPolygon2d supportPolygonToSet)
   {
      supportPolygonToSet.setIncludingFrame(supportPolygon);
   }

   public ContactStateEnum getContactState()
   {
      return state;
   }

   public double getInitialTime()
   {
      return initialTime;
   }

   public double getFinalTime()
   {
      return finalTime;
   }
}