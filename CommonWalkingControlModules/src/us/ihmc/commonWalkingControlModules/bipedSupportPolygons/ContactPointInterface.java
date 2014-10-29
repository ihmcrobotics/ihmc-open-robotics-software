package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import javax.vecmath.Point2d;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface ContactPointInterface
{
   public abstract boolean isInContact();

   public abstract void setInContact(boolean inContact);

   public abstract void getPosition(FramePoint framePointToPack);
   
   public abstract void getPosition2d(FramePoint2d framePoint2dToPack);

   public abstract void getPosition2d(Point2d position2d);

   public abstract void setPosition(FramePoint position);

   public abstract void setPosition2d(FramePoint2d position2d);

   public abstract ReferenceFrame getReferenceFrame();

   public abstract PlaneContactState getParentContactState();
}
