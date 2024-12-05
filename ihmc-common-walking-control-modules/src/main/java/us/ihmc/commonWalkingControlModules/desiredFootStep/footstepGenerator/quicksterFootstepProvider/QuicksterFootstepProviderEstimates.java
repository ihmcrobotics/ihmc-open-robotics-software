package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.AngularExcursionCalculator;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
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

   // Calculator for CoM momentum info
   private final AngularExcursionCalculator angularExcursionCalculator;

   public  QuicksterFootstepProviderEstimates(FullHumanoidRobotModel robotModel, CommonHumanoidReferenceFrames referenceFrames, FrameQuaternionReadOnly desiredPelvisOrientation, double updateDT, String variableNameSuffix, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
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
   }

   public void update()
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
