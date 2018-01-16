package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

public interface ContactPointInterface
{
   public abstract boolean isInContact();

   public abstract void setInContact(boolean inContact);

   public abstract void getPosition(FramePoint3D framePointToPack);

   public abstract FramePoint3D getPosition();

   public abstract void getPosition2d(FrameTuple2D<?, ?> framePoint2dToPack);

   public abstract void getPosition2d(Tuple2DBasics position2d);

   public abstract void setPosition(FrameTuple3DReadOnly position);

   public abstract void setPosition2d(FrameTuple2DReadOnly position2d);

   public abstract ReferenceFrame getReferenceFrame();

   public abstract PlaneContactState getParentContactState();
}
