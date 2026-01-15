package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.AngularExcursionCalculator;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuicksterFootstepProviderEstimates
{
   // CoM State Estimates
   private final YoFramePoint3D currentCoMPosition;
   private final YoFrameVector3D currentCoMVelocity;
   private final YoFrameVector3D currentCoMLinearMomentum;
   private final YoFrameVector3D currentCoMAngularMomentum;

   // CoM Frames
   private final MovingReferenceFrame centerOfMassFrame;
   private final MovingReferenceFrame centerOfMassControlFrame;
   private final MovingZUpFrame centerOfMassControlZUpFrame;
   private final YoGraphicReferenceFrame centerOfMassControlZUpFrameGraphic;

   // Calculator for CoM momentum info
   private final AngularExcursionCalculator angularExcursionCalculator;

   //
   private final double mass;
   private final YoFramePoint3D acp;
   private final QuicksterFootstepProviderParameters parameters;

   public  QuicksterFootstepProviderEstimates(FullHumanoidRobotModel robotModel,
                                              CommonHumanoidReferenceFrames referenceFrames,
                                              FrameQuaternionReadOnly desiredPelvisOrientation,
                                              QuicksterFootstepProviderParameters parameters,
                                              DoubleProvider updateDT,
                                              String variableNameSuffix,
                                              YoRegistry registry,
                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.parameters = parameters;
      mass = robotModel.getTotalMass();

      currentCoMPosition = new YoFramePoint3D("currentCoMPosition" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
      currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
      currentCoMLinearMomentum = new YoFrameVector3D("currentCoMLinearMomentum" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
      currentCoMAngularMomentum = new YoFrameVector3D("currentCoMAngularMomentum" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);

      centerOfMassFrame = (MovingReferenceFrame) referenceFrames.getCenterOfMassFrame();

      centerOfMassControlFrame = new MovingReferenceFrame("centerOfMassControlFrame" + variableNameSuffix, ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            //            pelvisFrame.getTwistRelativeToOther(ReferenceFrame.getWorldFrame(), pelvisTwist);
            //            pelvisTwist.changeFrame(centerOfMassFrame); // FIXME we really want the rotation about the center of mass, relative to the world.

            twistRelativeToParentToPack.getLinearPart().setMatchingFrame(getCenterOfMassVelocity());
            //            twistRelativeToParentToPack.getAngularPart().setMatchingFrame(pelvisTwist.getAngularPart());
         }

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(currentCoMPosition);
            transformToParent.getRotation().set(desiredPelvisOrientation);
         }
      };

      centerOfMassControlZUpFrame = new MovingZUpFrame(centerOfMassControlFrame, "centerOfMassControlZUpFrame" + variableNameSuffix);
      angularExcursionCalculator = new AngularExcursionCalculator(centerOfMassFrame, robotModel.getElevator(), updateDT, registry, null);
      acp = new YoFramePoint3D("ACP_" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);

      if (yoGraphicsListRegistry != null)
      {
         centerOfMassControlZUpFrameGraphic = new YoGraphicReferenceFrame(centerOfMassControlZUpFrame, registry, false, 1.25);
         yoGraphicsListRegistry.registerYoGraphic("QFP", centerOfMassControlZUpFrameGraphic);

         YoGraphicPosition acpViz = new YoGraphicPosition("ACP", acp, 0.01, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistry.registerArtifact("QFP", acpViz.createArtifact());
      }
      else
      {
         centerOfMassControlZUpFrameGraphic = null;
      }
   }

   public void update(RobotSide swingSide)
   {
      // Update CoM position and velocity from CoM frame
      currentCoMPosition.setFromReferenceFrame(centerOfMassFrame);
      currentCoMVelocity.setMatchingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());

      // Update CoM momentum info
      angularExcursionCalculator.compute();
      currentCoMLinearMomentum.setMatchingFrame(angularExcursionCalculator.getLinearMomentum());
      currentCoMAngularMomentum.setMatchingFrame(angularExcursionCalculator.getAngularMomentum());

      // Update CoM control frames
      centerOfMassControlFrame.update();
      centerOfMassControlZUpFrame.update();
      if (centerOfMassControlZUpFrameGraphic != null)
         centerOfMassControlZUpFrameGraphic.update();

      // Update Measured ACP
      double wmh = parameters.getOmega(swingSide).getDoubleValue() * mass * parameters.getDesiredCoMHeight(swingSide).getDoubleValue();

      acp.setFromReferenceFrame(centerOfMassFrame);
      acp.scaleAdd(1.0 / parameters.getOmega(swingSide).getDoubleValue(),
                   getCenterOfMassVelocity(),
                   getCenterOfMassPosition());
      acp.addX(1.0 / wmh * getCenterOfMassAngularMomentum().getY());
      acp.addY(-1.0 / wmh * getCenterOfMassAngularMomentum().getX());
   }

   public FramePoint3DReadOnly getCenterOfMassPosition()
   {
      return currentCoMPosition;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return currentCoMVelocity;
   }

   public FrameVector3DReadOnly getCenterOfMassLinearMomentum()
   {
      return currentCoMLinearMomentum;
   }

   public FrameVector3DReadOnly getCenterOfMassAngularMomentum()
   {
      return currentCoMAngularMomentum;
   }

   public MovingZUpFrame getCenterOfMassControlZUpFrame()
   {
      return centerOfMassControlZUpFrame;
   }
}
