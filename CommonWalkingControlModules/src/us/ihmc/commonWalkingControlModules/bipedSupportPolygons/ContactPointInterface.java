package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.geometry.FramePoint2D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ContactPointInterface
{
   public abstract boolean isInContact();

   public abstract void setInContact(boolean inContact);

   public abstract void getPosition(FramePoint3D framePointToPack);

   public abstract FramePoint3D getPosition();

   public abstract void getPosition2d(FramePoint2D framePoint2dToPack);

   public abstract void getPosition2d(Point2D position2d);

   public abstract void setPosition(FramePoint3D position);

   public abstract void setPosition2d(FramePoint2D position2d);

   public abstract ReferenceFrame getReferenceFrame();

   public abstract PlaneContactState getParentContactState();
}
