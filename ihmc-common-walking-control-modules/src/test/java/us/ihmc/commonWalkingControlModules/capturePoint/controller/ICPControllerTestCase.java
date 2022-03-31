package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;

public class ICPControllerTestCase
{
   private double omega;

   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final FrameVector2D desiredICPVelocity = new FrameVector2D();
   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D perfectCoP = new FramePoint2D();
   private final FrameVector2D perfectCMPOffset = new FrameVector2D();
   private final FramePoint2D currentCoMPosition = new FramePoint2D();
   private final FramePoint2D currentICP = new FramePoint2D();

   private final FramePoint2D desiredCMP = new FramePoint2D();
   private final FramePoint2D desiredCoP = new FramePoint2D();
   private final FrameVector2D expectedControlICPVelocity = new FrameVector2D();
   
   public ICPControllerTestCase()
   {
      omega = Double.NaN;
      supportPolygonInWorld.setToNaN();
      desiredICPVelocity.setToNaN();
      desiredICP.setToNaN();
      perfectCoP.setToNaN();
      perfectCMPOffset.setToNaN();
      currentCoMPosition.setToNaN();
      currentICP.setToNaN();
      desiredCMP.setToNaN();
      desiredCoP.setToNaN();
      expectedControlICPVelocity.setToNaN();
   }

   public ICPControllerTestCase(ICPControllerTestCase testCase)
   {
      this.setOmega(testCase.getOmega());
      this.setSupportPolygonInWorld(testCase.getSupportPolygonInWorld());
      this.setDesiredICPVelocity(testCase.getDesiredICPVelocity());
      this.setDesiredICP(testCase.getDesiredICP());
      this.setPerfectCoP(testCase.getPerfectCoP());
      this.setPerfectCMPOffset(testCase.getPerfectCMPOffset());
      this.setCurrentCoMPosition(testCase.getCurrentCoMPosition());
      this.setCurrentICP(testCase.getCurrentICP());

      desiredCMP.setToNaN();
      desiredCoP.setToNaN();
      expectedControlICPVelocity.setToNaN();
   }

   public double getOmega()
   {
      return omega;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public FrameConvexPolygon2DReadOnly getSupportPolygonInWorld()
   {
      return supportPolygonInWorld;
   }

   public void setSupportPolygonInWorld(FrameConvexPolygon2DReadOnly supportPolygonInWorld)
   {
      this.supportPolygonInWorld.set(supportPolygonInWorld);
   }

   public FrameVector2DReadOnly getDesiredICPVelocity()
   {
      return desiredICPVelocity;
   }

   public void setDesiredICPVelocity(FrameVector2DReadOnly desiredICPVelocity)
   {
      this.desiredICPVelocity.set(desiredICPVelocity);
   }

   public FramePoint2DReadOnly getDesiredICP()
   {
      return desiredICP;
   }

   public void setDesiredICP(FramePoint2DReadOnly desiredICP)
   {
      this.desiredICP.set(desiredICP);
   }

   public FramePoint2DReadOnly getPerfectCoP()
   {
      return perfectCoP;
   }

   public void setPerfectCoP(FramePoint2DReadOnly perfectCoP)
   {
      this.perfectCoP.set(perfectCoP);
   }

   public FrameVector2D getPerfectCMPOffset()
   {
      return perfectCMPOffset;
   }

   public void setPerfectCMPOffset(FrameVector2DReadOnly perfectCMPOffset)
   {
      this.perfectCMPOffset.set(perfectCMPOffset);
   }

   public FramePoint2DReadOnly getCurrentCoMPosition()
   {
      return currentCoMPosition;
   }

   public void setCurrentCoMPosition(FramePoint2DReadOnly currentCoMPosition)
   {
      this.currentCoMPosition.set(currentCoMPosition);
   }

   public FramePoint2DReadOnly getCurrentICP()
   {
      return currentICP;
   }

   public void setCurrentICP(FramePoint2DReadOnly currentICP)
   {
      this.currentICP.set(currentICP);
   }

   public FramePoint2DReadOnly getDesiredCMP()
   {
      return desiredCMP;
   }

   public void setDesiredCMP(FramePoint2DReadOnly desiredCMP)
   {
      this.desiredCMP.set(desiredCMP);
   }

   public FramePoint2DReadOnly getDesiredCoP()
   {
      return desiredCoP;
   }

   public void setDesiredCoP(FramePoint2DReadOnly desiredCoP)
   {
      this.desiredCoP.set(desiredCoP);
   }

   public FrameVector2DReadOnly getExpectedControlICPVelocity()
   {
      return expectedControlICPVelocity;
   }

   public void setExpectedControlICPVelocity(FrameVector2D expectedControlICPVelocity)
   {
      this.expectedControlICPVelocity.set(expectedControlICPVelocity);
   }
   
}
