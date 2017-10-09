package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;

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
