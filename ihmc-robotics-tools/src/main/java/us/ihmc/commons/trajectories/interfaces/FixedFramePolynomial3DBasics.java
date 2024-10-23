package us.ihmc.commons.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface FixedFramePolynomial3DBasics extends FramePolynomial3DReadOnly, Polynomial3DBasics, ReferenceFrameHolder
{
   default void set(FramePolynomial3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, Polynomial3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      Polynomial3DBasics.super.set(other);
   }

   default void setConstant(FramePoint3DReadOnly z)
   {
      checkReferenceFrameMatch(z);
      Polynomial3DBasics.super.setConstant(z);
   }

   default void setCubic(double t0, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zFinal);
      Polynomial3DBasics.super.setCubic(t0, tFinal, z0, zFinal);
   }

   default void setCubic(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setCubic(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }

   default void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal, FrameVector3DReadOnly zddFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);
      checkReferenceFrameMatch(zddFinal);

      Polynomial3DBasics.super.setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0, zFinal, zdFinal, zddFinal);
   }

   default void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zFinal);

      Polynomial3DBasics.super.setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0, zd0, zdd0, zFinal);
   }

   default void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdFinal, FrameVector3DReadOnly zddFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdFinal);
      checkReferenceFrameMatch(zddFinal);

      Polynomial3DBasics.super.setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0, zd0, zdFinal, zddFinal);
   }

   default void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zIntermediate1,
                                                FramePoint3DReadOnly zIntermediate2, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zIntermediate2);
      checkReferenceFrameMatch(zFinal);

      Polynomial3DBasics.super.setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0, zIntermediate1, zIntermediate2, zFinal);
   }

   default void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zIntermediate1, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zFinal);

      Polynomial3DBasics.super.setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0, zIntermediate1, zFinal);
   }

   default void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0,
                                                                           FramePoint3DReadOnly zIntermediate, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0, zIntermediate, zFinal, zdFinal);
   }

   default void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0,
                                                                             FramePoint3DReadOnly zIntermediate, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zFinal);

      Polynomial3DBasics.super.setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate, zFinal);
   }

   default void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }

   default void setLinear(double t0, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zf);

      Polynomial3DBasics.super.setLinear(t0, tFinal, z0, zf);
   }

   default void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zIntermediate0,
                        FrameVector3DReadOnly zdIntermediate0, FramePoint3DReadOnly zIntermediate1, FrameVector3DReadOnly zdIntermediate1, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zIntermediate0);
      checkReferenceFrameMatch(zdIntermediate0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zdIntermediate1);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);

      Polynomial3DBasics.super.setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zdIntermediate0,
               zIntermediate1, zdIntermediate1, zf, zdf);
   }

   default void setQuadratic(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zFinal);

      Polynomial3DBasics.super.setQuadratic(t0, tFinal, z0, zd0, zFinal);
   }

   default void setQuadraticUsingInitialAcceleration(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);

      Polynomial3DBasics.super.setQuadraticUsingInitialAcceleration(t0, tFinal, z0, zd0, zdd0);
   }

   default void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zIntermediate, FramePoint3DReadOnly zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zFinal);

      Polynomial3DBasics.super.setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0, zIntermediate, zFinal);
   }

   default void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, FramePoint3DReadOnly z0, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0, zFinal, zdFinal);
   }

   default void setQuartic(double t0, double tFinal, FramePoint3D z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setQuartic(t0, tFinal, z0, zd0, zdd0, zFinal, zdFinal);
   }

   default void setQuarticUsingFinalAcceleration(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal, FrameVector3DReadOnly zddFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);
      checkReferenceFrameMatch(zddFinal);

      Polynomial3DBasics.super.setQuarticUsingFinalAcceleration(t0, tFinal, z0, zd0, zFinal, zdFinal, zddFinal);
   }

   default void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdIntermediate,
                                                    FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdIntermediate);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0, zd0, zdIntermediate, zFinal, zdFinal);
   }

   default void setQuarticUsingMidPoint(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zMid, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zMid);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setQuarticUsingMidPoint(t0, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   default void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3DReadOnly z0,
                                                       FramePoint3DReadOnly zIntermediate0, FramePoint3DReadOnly zIntermediate1, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdIntermediate1)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zIntermediate0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdIntermediate1);

      Polynomial3DBasics.super.setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0, zIntermediate0, zIntermediate1,
                                             zFinal, zdIntermediate1);
   }

   default void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zIntermediate, FramePoint3DReadOnly zf,
                                       FrameVector3DReadOnly zdf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);

      Polynomial3DBasics.super.setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate, zf, zdf);
   }

   default void setQuintic(double t0, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf, FrameVector3DReadOnly zddf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);
      checkReferenceFrameMatch(zddf);

      Polynomial3DBasics.super.setQuintic(t0, tFinal, z0, zd0, zdd0, zf, zdf, zddf);
   }

   default void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0,
                                       FramePoint3DReadOnly zIntermediate0, FramePoint3DReadOnly zIntermediate1, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zIntermediate0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);

      Polynomial3DBasics.super.setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zIntermediate1,
                             zf, zdf);
   }

   default void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0,
                                                                   FrameVector3DReadOnly zdIntermediate, FrameVector3DReadOnly zddIntermediate, FramePoint3DReadOnly zFinal,
                                                                  FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdIntermediate);
      checkReferenceFrameMatch(zddIntermediate);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0, zd0, zdIntermediate,
                                                         zddIntermediate, zFinal, zdFinal);
   }

   default void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0,
                                        FramePoint3DReadOnly zIntermediate, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);

      Polynomial3DBasics.super.setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf, zdf);
   }

   default void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0,
                                         FramePoint3DReadOnly zIntermediate, FrameVector3DReadOnly zdIntermediate, FramePoint3DReadOnly zf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zdIntermediate);
      checkReferenceFrameMatch(zf);

      Polynomial3DBasics.super.setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate,
                               zdIntermediate, zf);
   }

   default void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FramePoint3DReadOnly zIntermediate0,
                         FrameVector3DReadOnly zdIntermediate0, FramePoint3DReadOnly zIntermediate1, FrameVector3DReadOnly zdIntermediate1, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zIntermediate0);
      checkReferenceFrameMatch(zdIntermediate0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zdIntermediate1);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);

      Polynomial3DBasics.super.setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zdIntermediate0,
                zIntermediate1, zdIntermediate1, zf, zdf);
   }

   default void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0,
                                                    FrameVector3DReadOnly zdd0, FramePoint3DReadOnly zIntermediate0, FramePoint3DReadOnly zIntermediate1, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf,
                                                    FrameVector3DReadOnly zddf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zIntermediate0);
      checkReferenceFrameMatch(zIntermediate1);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);
      checkReferenceFrameMatch(zddf);

      Polynomial3DBasics.super.setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zdd0,
                                           zIntermediate0, zIntermediate1, zf, zdf, zddf);
   }

   default void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0,
                                       FramePoint3DReadOnly zIntermediate, FramePoint3DReadOnly zf, FrameVector3DReadOnly zdf, FrameVector3DReadOnly zddf)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zIntermediate);
      checkReferenceFrameMatch(zf);
      checkReferenceFrameMatch(zdf);
      checkReferenceFrameMatch(zddf);

      Polynomial3DBasics.super.setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf,
                             zdf, zddf);
   }

   default void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint3DReadOnly z0, FrameVector3DReadOnly zd0, FrameVector3DReadOnly zdd0,
                                                             FrameVector3DReadOnly zdIntermediate, FrameVector3DReadOnly zddIntermediate, FramePoint3DReadOnly zFinal, FrameVector3DReadOnly zdFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zd0);
      checkReferenceFrameMatch(zdd0);
      checkReferenceFrameMatch(zdIntermediate);
      checkReferenceFrameMatch(zddIntermediate);
      checkReferenceFrameMatch(zFinal);
      checkReferenceFrameMatch(zdFinal);

      Polynomial3DBasics.super.setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0, zd0, zdd0, zdIntermediate,
                                                    zddIntermediate, zFinal, zdFinal);
   }



}
