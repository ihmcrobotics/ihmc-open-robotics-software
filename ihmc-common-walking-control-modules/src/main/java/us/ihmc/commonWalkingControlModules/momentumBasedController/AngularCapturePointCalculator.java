package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.humanoidRobotics.model.CentroidalMomentumProvider;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

import java.util.function.Supplier;

public class AngularCapturePointCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CenterOfMassStateProvider centerOfMassStateProvider;
   private final CentroidalMomentumProvider centroidalMomentumProvider;
   private final double gravityZ;
   private final double totalMass;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();
   private final FrameVector2D centerOfMassVelocity2d = new FrameVector2D();
   private final FrameVector2D equivalentCenterOfMassVelocity2d = new FrameVector2D();

   public AngularCapturePointCalculator(CenterOfMassStateProvider centerOfMassStateProvider,
                                        CentroidalMomentumProvider centroidalMomentumProvider,
                                        double gravityZ,
                                        double totalMass)
   {
      this.centerOfMassStateProvider = centerOfMassStateProvider;
      this.centroidalMomentumProvider = centroidalMomentumProvider;
      this.gravityZ = gravityZ;
      this.totalMass = totalMass;
   }

   private final FrameVector3DBasics centroidalAngularMomentum = new FrameVector3D();

   public void compute(FramePoint2DBasics angularCapturePointToPack, FramePoint2DBasics capturePointToPack, double omega0)
   {
      centerOfMassPosition.setIncludingFrame(centerOfMassStateProvider.getCenterOfMassPosition());
      centerOfMassVelocity.setIncludingFrame(centerOfMassStateProvider.getCenterOfMassVelocity());

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      centerOfMassVelocity2d.setIncludingFrame(centerOfMassVelocity);

      CapturePointTools.computeCapturePointPosition(centerOfMassPosition2d, centerOfMassVelocity2d, omega0, capturePointToPack);

      // scale the angular momentum by g and z to get the velocity;
      double deltaZ = gravityZ / MathTools.square(omega0);
      centroidalAngularMomentum.setIncludingFrame(centroidalMomentumProvider.getCentroidalAngularMomentum());
      centroidalAngularMomentum.changeFrame(worldFrame);

      double equivalentXVelocity = centroidalAngularMomentum.getY() / (deltaZ * totalMass);
      double equivalentYVelocity = -centroidalAngularMomentum.getX() / (deltaZ * totalMass);

      equivalentCenterOfMassVelocity2d.set(centerOfMassVelocity2d);
      equivalentCenterOfMassVelocity2d.add(equivalentXVelocity, equivalentYVelocity);

      CapturePointTools.computeCapturePointPosition(centerOfMassPosition2d, equivalentCenterOfMassVelocity2d, omega0, angularCapturePointToPack);
   }

   public FramePoint3DReadOnly getCenterOfMassPosition()
   {
      return centerOfMassPosition;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return centerOfMassVelocity;
   }
}
