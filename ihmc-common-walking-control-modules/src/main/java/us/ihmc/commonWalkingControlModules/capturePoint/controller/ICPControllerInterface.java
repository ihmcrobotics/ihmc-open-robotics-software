package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;

public interface ICPControllerInterface
{
   public abstract void initialize();

   public abstract void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon);

   public abstract void compute(FramePoint2DReadOnly desiredICP,
                                FrameVector2DReadOnly desiredICPVelocity,
                                FramePoint2DReadOnly perfectCoP,
                                FramePoint2DReadOnly currentICP,
                                FramePoint2DReadOnly currentCoMPosition,
                                double omega0);

   public abstract void compute(FramePoint2DReadOnly desiredICP,
                                FrameVector2DReadOnly desiredICPVelocity,
                                FramePoint2DReadOnly perfectCoP,
                                FrameVector2DReadOnly perfectCMPOffset,
                                FramePoint2DReadOnly currentICP,
                                FramePoint2DReadOnly currentCoMPosition,
                                double omega0);


   public abstract void getDesiredCMP(FixedFramePoint2DBasics desiredCMPToPack);

   public abstract void getDesiredCoP(FixedFramePoint2DBasics desiredCoPToPack);

   public abstract boolean useAngularMomentum();

   public abstract FrameVector2DReadOnly getResidualError();

}
