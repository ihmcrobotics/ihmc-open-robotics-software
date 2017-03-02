package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ContactPointInterface
{
   public abstract boolean isInContact();

   public abstract void setInContact(boolean inContact);

   public abstract void getPosition(FramePoint framePointToPack);
   
   public abstract void getPosition2d(FramePoint2d framePoint2dToPack);

   public abstract void getPosition2d(Point2D position2d);

   public abstract void setPosition(FramePoint position);

   public abstract void setPosition2d(FramePoint2d position2d);

   public abstract ReferenceFrame getReferenceFrame();

   public abstract PlaneContactState getParentContactState();
}
