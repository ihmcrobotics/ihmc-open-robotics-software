package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MPTCSpatialFeedbackController extends SpatialFeedbackController
{
   private final MPTCGainsCalculator mptcGainsCalculator;
   private final YoMatrix transformedProportionalGains;
   private final YoMatrix transformedDerivativeGains;

   public MPTCSpatialFeedbackController(RigidBodyBasics endEffector, int controllerIndex, WholeBodyControlCoreToolbox ccToolbox,
                                        FeedbackControllerToolbox fbToolbox, YoRegistry parentRegistry)
   {
      super(endEffector, controllerIndex, ccToolbox, fbToolbox, parentRegistry);
      mptcGainsCalculator = new MPTCGainsCalculator(MultiBodySystemTools.getRootBody(endEffector));
      transformedProportionalGains = new YoMatrix(endEffector.getName() + "SpatialFBTransformedProportionalGains", 6, 6, registry);
      transformedDerivativeGains = new YoMatrix(endEffector.getName() + "SpatialFBTransformedDerivativeGains", 6, 6, registry);
   }

   @Override
   public void computeInverseDynamics()
   {
      mptcGainsCalculator.computeMPTCGains(gains, endEffector, controlFrame);
      super.computeInverseDynamics();
   }

   private final DMatrixRMaj spatialError = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj spatialFeedbackTerm = new DMatrixRMaj(6, 1);

   @Override
   protected void computeProportionalTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredPose.getReferenceFrame();

      yoCurrentPose.setToZero(controlFrame);
      yoCurrentPose.changeFrame(trajectoryFrame);
      yoCurrentPose.setCommandId(currentCommandId);
      yoCurrentPose.getOrientation().getRotationVector(yoCurrentRotationVector);
      yoCurrentRotationVector.setCommandId(currentCommandId);

      desiredPose.setIncludingFrame(yoDesiredPose);
      desiredPose.changeFrame(controlFrame);

      desiredPose.getOrientation().normalizeAndLimitToPi();
      linearFeedbackTermToPack.setIncludingFrame(desiredPose.getPosition());
      desiredPose.getRotationVector(angularFeedbackTermToPack);

      selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
      selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);

      linearFeedbackTermToPack.clipToMaxLength(positionGains.getMaximumProportionalError());
      angularFeedbackTermToPack.clipToMaxLength(orientationGains.getMaximumProportionalError());

      yoErrorVector.setIncludingFrame(angularFeedbackTermToPack, linearFeedbackTermToPack);
      yoErrorVector.changeFrame(trajectoryFrame);
      yoErrorVector.setCommandId(currentCommandId);
      yoErrorOrientation.setRotationVectorIncludingFrame(yoErrorVector.getAngularPart());
      yoErrorOrientation.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         linearFeedbackTermToPack.changeFrame(linearGainsFrame);
      else
         linearFeedbackTermToPack.changeFrame(controlFrame);

      if (angularGainsFrame != null)
         angularFeedbackTermToPack.changeFrame(angularGainsFrame);
      else
         angularFeedbackTermToPack.changeFrame(controlFrame);

      angularFeedbackTermToPack.get(0, spatialError);
      linearFeedbackTermToPack.get(3, spatialError);
      CommonOps_DDRM.mult(mptcGainsCalculator.getProportionalGains(), spatialError, spatialFeedbackTerm);
      transformedProportionalGains.set(mptcGainsCalculator.getProportionalGains());

      angularFeedbackTermToPack.set(0, spatialFeedbackTerm);
      linearFeedbackTermToPack.set(3, spatialFeedbackTerm);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
   }

   @Override
   protected void computeDerivativeTerm(FrameVector3D linearFeedbackTermToPack, FrameVector3D angularFeedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredPose.getReferenceFrame();

      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      yoCurrentVelocity.setIncludingFrame(currentTwist.getAngularPart(), currentTwist.getLinearPart());
      yoCurrentVelocity.changeFrame(trajectoryFrame);
      yoCurrentVelocity.setCommandId(currentCommandId);

      linearFeedbackTermToPack.setToZero(trajectoryFrame);
      angularFeedbackTermToPack.setToZero(trajectoryFrame);
      linearFeedbackTermToPack.sub(yoDesiredVelocity.getLinearPart(), yoCurrentVelocity.getLinearPart());
      angularFeedbackTermToPack.sub(yoDesiredVelocity.getAngularPart(), yoCurrentVelocity.getAngularPart());
      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyLinearSelection(linearFeedbackTermToPack);
      selectionMatrix.applyAngularSelection(angularFeedbackTermToPack);

      linearFeedbackTermToPack.clipToMaxLength(positionGains.getMaximumDerivativeError());
      angularFeedbackTermToPack.clipToMaxLength(orientationGains.getMaximumDerivativeError());

      if (yoFilteredErrorVelocity != null)
      {
         // If the trajectory frame changed reset the filter.
         if (yoFilteredErrorVelocity.getReferenceFrame() != trajectoryFrame)
         {
            yoFilteredErrorVelocity.setReferenceFrame(trajectoryFrame);
            yoFilteredErrorVelocity.reset();
         }
         linearFeedbackTermToPack.changeFrame(trajectoryFrame);
         angularFeedbackTermToPack.changeFrame(trajectoryFrame);
         yoErrorVelocity.setIncludingFrame(angularFeedbackTermToPack, linearFeedbackTermToPack);
         yoFilteredErrorVelocity.update();
         yoFilteredErrorVelocity.setCommandId(currentCommandId);
         linearFeedbackTermToPack.set(yoFilteredErrorVelocity.getLinearPart());
         angularFeedbackTermToPack.set(yoFilteredErrorVelocity.getAngularPart());
      }
      else
      {
         yoErrorVelocity.setIncludingFrame(angularFeedbackTermToPack, linearFeedbackTermToPack);
      }
      yoErrorVelocity.changeFrame(trajectoryFrame);
      yoErrorVelocity.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         linearFeedbackTermToPack.changeFrame(linearGainsFrame);
      else
         linearFeedbackTermToPack.changeFrame(controlFrame);

      if (angularGainsFrame != null)
         angularFeedbackTermToPack.changeFrame(angularGainsFrame);
      else
         angularFeedbackTermToPack.changeFrame(controlFrame);

      angularFeedbackTermToPack.get(0, spatialError);
      linearFeedbackTermToPack.get(3, spatialError);
      CommonOps_DDRM.mult(mptcGainsCalculator.getDerivativeGains(), spatialError, spatialFeedbackTerm);
      transformedDerivativeGains.set(mptcGainsCalculator.getDerivativeGains());

      angularFeedbackTermToPack.set(0, spatialFeedbackTerm);
      linearFeedbackTermToPack.set(3, spatialFeedbackTerm);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
   }
}
