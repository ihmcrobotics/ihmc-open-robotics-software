package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ContactPoint implements ContactPointInterface
{
   private boolean inContact = false;
   private final FramePoint position;
   private final PlaneContactState parentContactState;

   public ContactPoint(FramePoint2d point2d, PlaneContactState parentContactState)
   {
      position = new FramePoint(point2d.getReferenceFrame(), point2d.getX(), point2d.getY(), 0.0);
      this.parentContactState = parentContactState;
   }

   @Override
   public boolean isInContact()
   {
      return inContact;
   }

   @Override
   public void setInContact(boolean inContact)
   {
      this.inContact = inContact;
   }

   @Override
   public void setPosition(FramePoint position)
   {
      this.position.set(position);
   }

   @Override
   public PlaneContactState getParentContactState()
   {
      return parentContactState;
   }

   @Override
   public void getPosition(FramePoint framePointToPack)
   {
      framePointToPack.setIncludingFrame(position);
   }

   @Override
   public void getPosition2d(FramePoint2d framePoint2dToPack)
   {
      framePoint2dToPack.setIncludingFrame(getReferenceFrame(), position.getX(), position.getY());
   }

   @Override
   public void getPosition2d(Point2D position2d)
   {
      position.setXY(position2d);
   }

   @Override
   public void setPosition2d(FramePoint2d position2d)
   {
      position.setXY(position2d);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }
}
