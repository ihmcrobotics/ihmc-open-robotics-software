package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ALIPController
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoFrameVector2D currentALIPMomentum = new YoFrameVector2D("currentALIPMomentum", worldFrame, registry);
   private final YoFrameVector2D desiredALIPMomentum = new YoFrameVector2D("desiredALIPMomentum", worldFrame, registry);
   private final YoFrameVector2D desiredALIPMomentumRate = new YoFrameVector2D("desiredALIPMomentumRate", worldFrame, registry);

   private final YoFrameVector2D alipError = new YoFrameVector2D("alipError", worldFrame, registry);
   private final YoFrameVector2D alipFeedback = new YoFrameVector2D("alipFeedback", worldFrame, registry);

   private final YoFramePoint2D alipFeedbackCoP = new YoFramePoint2D("alipFeedbackCoP", worldFrame, registry);

   private final YoDouble alipGain = new YoDouble("alipGain", registry);

   private final FramePoint2D tempPoint = new FramePoint2D();

   private final double gravityZ;
   private final double mass;

   public ALIPController(double gravityZ, double mass, YoRegistry parentRegistry)
   {
      this.gravityZ = Math.abs(gravityZ);
      this.mass = mass;

      alipGain.set(1.5);

      parentRegistry.addChild(registry);
   }

   public void compute(FramePoint3DReadOnly centerOfMassPosition,
                       FrameVector3DReadOnly centerOfMassVelocity,
                       FrameVector3DReadOnly centroidalAngularMomentum,
                       FramePoint3DReadOnly desiredCenterOfMassPosition,
                       FrameVector3DReadOnly desiredCenterOfMassVelocity,
                       FramePoint2DReadOnly desiredCenterOfPressurePosition,
                       double omega)
   {
      centerOfMassPosition.checkReferenceFrameMatch(worldFrame);
      centerOfMassVelocity.checkReferenceFrameMatch(worldFrame);
      centroidalAngularMomentum.checkReferenceFrameMatch(worldFrame);
      desiredCenterOfMassPosition.checkReferenceFrameMatch(worldFrame);
      desiredCenterOfMassVelocity.checkReferenceFrameMatch(worldFrame);
      desiredCenterOfPressurePosition.checkReferenceFrameMatch(worldFrame);

      double desiredXMomentArm = desiredCenterOfMassPosition.getX() - desiredCenterOfPressurePosition.getX();
      double desiredYMomentArm = desiredCenterOfMassPosition.getY() - desiredCenterOfPressurePosition.getY();

      double weight = mass * gravityZ;
      double height = gravityZ / MathTools.square(omega);

      desiredALIPMomentum.setY(desiredCenterOfMassVelocity.getX() * mass * height);
      desiredALIPMomentum.setX(-desiredCenterOfMassVelocity.getY() * mass * height);

      desiredALIPMomentumRate.setY(desiredXMomentArm * weight);
      desiredALIPMomentumRate.setX(-desiredYMomentArm * weight);

      currentALIPMomentum.setY(centerOfMassVelocity.getX() * mass * height);
      currentALIPMomentum.setX(-centerOfMassVelocity.getY() * mass * height);
      currentALIPMomentum.add(centroidalAngularMomentum.getX(), centroidalAngularMomentum.getY());

      alipError.sub(desiredALIPMomentum, currentALIPMomentum);
      alipFeedback.setAndScale(alipGain.getValue(), alipError);

      tempPoint.add(alipFeedback, desiredALIPMomentumRate);

      alipFeedbackCoP.set(tempPoint.getY(), -tempPoint.getX());
      alipFeedbackCoP.scale(-1.0 / weight);
      alipFeedbackCoP.add(centerOfMassPosition.getX(), centerOfMassPosition.getY());
   }

   public FramePoint2DReadOnly getDesiredCoP()
   {
      return alipFeedbackCoP;
   }
}
