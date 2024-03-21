package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;

public class ALIPTools
{
   public static void computeCenterOfMassPosition(double time,
                                              double omega,
                                              double mass,
                                              double gravity,
                                              FramePoint2DReadOnly initialCoMPosition,
                                              FrameVector2DReadOnly initialAngularMomentum,
                                              FramePoint2DReadOnly pendulumBase,
                                              FramePoint2DBasics centerOfMassPositionToPack)
   {
      // Verify everything is in the same reference frame
      initialCoMPosition.checkReferenceFrameMatch(initialAngularMomentum);
      initialCoMPosition.checkReferenceFrameMatch(pendulumBase);
      initialCoMPosition.checkReferenceFrameMatch(centerOfMassPositionToPack);

      double desiredHeight = gravity / (omega * omega);
      double wT = omega * time;
      double coshWt = Math.cosh(wT);
      double sinhWt = Math.sinh(wT);
      double equivalentVelocityX = initialAngularMomentum.getY() / (mass * desiredHeight);
      double equivalentVelocityY = -initialAngularMomentum.getX() / (mass * desiredHeight);

      // Why this is an interpolation: cosh(wt)*(x - r) + r  ==>  cosh(wt)*x + (1 - cosh(wt))*r  ==>  a*x + (1 - a)*r where a = cosh(wt)
      centerOfMassPositionToPack.interpolate(pendulumBase, initialCoMPosition, coshWt);

      centerOfMassPositionToPack.addX(sinhWt / omega * equivalentVelocityX);
      centerOfMassPositionToPack.addY(sinhWt / omega * equivalentVelocityY);
   }

   public static void computeAngularMomentum(double time,
                                             double omega,
                                             double mass,
                                             double gravity,
                                             FramePoint2DReadOnly initialCoMPosition,
                                             FrameVector2DReadOnly initialAngularMomentum,
                                             FramePoint2DReadOnly pendulumBase,
                                             FrameVector2DBasics angularMomentumToPack)
   {
      // Verify everything is in the same reference frame
      initialCoMPosition.checkReferenceFrameMatch(initialAngularMomentum);
      initialCoMPosition.checkReferenceFrameMatch(pendulumBase);
      initialCoMPosition.checkReferenceFrameMatch(angularMomentumToPack);

      double desiredHeight = gravity / (omega * omega);
      double wT = omega * time;
      double coshWt = Math.cosh(wT);
      double sinhWt = Math.sinh(wT);
      double relativeCoMPositionX = initialCoMPosition.getX() - pendulumBase.getX();
      double relativeCoMPositionY = initialCoMPosition.getY() - pendulumBase.getY();
//      return (-terminalComPosition * sinhWt * totalMass * desiredHeight * omega + terminalAngularMomentum * coshWt) / determinant;

//      angularMomentumToPack.interpolate(pendulumBase, initialCoMPosition, mass * desiredHeight * omega * sinhWt);
      angularMomentumToPack.setX(-relativeCoMPositionY * mass * desiredHeight * omega * sinhWt + initialAngularMomentum.getX() * coshWt);
      angularMomentumToPack.setY(relativeCoMPositionX * mass * desiredHeight * omega * sinhWt + initialAngularMomentum.getY() * coshWt);
   }

   public static void computeACP(double omega,
                                 double mass,
                                 double gravity,
                                 FramePoint2DReadOnly comPosition,
                                 FrameVector2DReadOnly angularMomentumAboutContact,
                                 FramePoint2DBasics acpToPack)
   {
      double desiredHeight = gravity / (omega * omega);

      double equivalentVelocityX = angularMomentumAboutContact.getY() / (mass * desiredHeight);
      double equivalentVelocityY = -angularMomentumAboutContact.getX() / (mass * desiredHeight);

      acpToPack.setReferenceFrame(angularMomentumAboutContact.getReferenceFrame());
      acpToPack.set(equivalentVelocityX, equivalentVelocityY);
      acpToPack.scaleAdd(1.0 / omega, comPosition);
   }



}
