package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;

public interface ICPControllerInterface extends SCS2YoGraphicHolder
{
   void initialize();

   void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon);

   default void compute(FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                        FramePoint2DReadOnly desiredICP,
                        FrameVector2DReadOnly desiredICPVelocity,
                        FramePoint2DReadOnly finalICP,
                        FramePoint2DReadOnly perfectCoP,
                        FramePoint2DReadOnly currentICP,
                        FramePoint2DReadOnly currentCoMPosition,
                        double omega0)
   {
      compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, finalICP, perfectCoP, null, currentICP, currentCoMPosition, omega0);
   }

   void compute(FrameConvexPolygon2DReadOnly supportPolygonInWorld,
                FramePoint2DReadOnly desiredICP,
                FrameVector2DReadOnly desiredICPVelocity,
                FramePoint2DReadOnly finalICP,
                FramePoint2DReadOnly perfectCoP,
                FrameVector2DReadOnly perfectCMPOffset,
                FramePoint2DReadOnly currentICP,
                FramePoint2DReadOnly currentCoMPosition,
                double omega0);

   FramePoint2DReadOnly getDesiredCMP();

   FramePoint2DReadOnly getDesiredCoP();

   FrameVector2DReadOnly getExpectedControlICPVelocity();

   boolean useAngularMomentum();
}
