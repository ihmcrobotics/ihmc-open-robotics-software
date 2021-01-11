package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MPTCOrientationFeedbackController extends OrientationFeedbackController
{
   private final MPTCGainsCalculator mptcGainsCalculator;

   public MPTCOrientationFeedbackController(RigidBodyBasics endEffector, int controllerIndex, WholeBodyControlCoreToolbox ccToolbox,
                                            FeedbackControllerToolbox fbToolbox, YoRegistry parentRegistry)
   {
      super(endEffector, controllerIndex, ccToolbox, fbToolbox, parentRegistry);
      mptcGainsCalculator = new MPTCGainsCalculator(MultiBodySystemTools.getRootBody(endEffector));
   }

   @Override
   public void computeInverseDynamics()
   {
      mptcGainsCalculator.computeMPTCGains(gains, null, endEffector, endEffectorFrame);
      super.computeInverseDynamics();
   }

   private final DMatrixRMaj spatialError = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj spatialFeedbackTerm = new DMatrixRMaj(6, 1);

   @Override
   protected void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      yoCurrentOrientation.setToZero(endEffectorFrame);
      yoCurrentOrientation.changeFrame(trajectoryFrame);
      yoCurrentOrientation.setCommandId(currentCommandId);
      yoCurrentOrientation.getRotationVector(yoCurrentRotationVector);
      yoCurrentRotationVector.setCommandId(currentCommandId);

      desiredOrientation.setIncludingFrame(yoDesiredOrientation);
      desiredOrientation.changeFrame(endEffectorFrame);

      desiredOrientation.normalizeAndLimitToPi();
      desiredOrientation.getRotationVector(feedbackTermToPack);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumProportionalError());

      yoErrorRotationVector.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVector.changeFrame(trajectoryFrame);
      yoErrorRotationVector.setCommandId(currentCommandId);
      yoErrorOrientation.setRotationVectorIncludingFrame(yoErrorRotationVector);
      yoErrorRotationVector.setCommandId(currentCommandId);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(endEffectorFrame);

      feedbackTermToPack.get(0, spatialError);
      CommonOps_DDRM.mult(mptcGainsCalculator.getProportionalGains(), spatialError, spatialFeedbackTerm);
      feedbackTermToPack.set(0, spatialFeedbackTerm);

      feedbackTermToPack.changeFrame(endEffectorFrame);
   }

   @Override
   public void computeDerivativeTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      endEffectorFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      yoCurrentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      yoCurrentAngularVelocity.changeFrame(trajectoryFrame);
      yoCurrentAngularVelocity.setCommandId(currentCommandId);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredAngularVelocity, yoCurrentAngularVelocity);
      feedbackTermToPack.changeFrame(endEffectorFrame);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumDerivativeError());

      if (yoFilteredErrorAngularVelocity != null)
      {
         // If the trajectory frame changed reset the filter.
         if (yoFilteredErrorAngularVelocity.getReferenceFrame() != trajectoryFrame)
         {
            yoFilteredErrorAngularVelocity.setReferenceFrame(trajectoryFrame);
            yoFilteredErrorAngularVelocity.reset();
         }
         feedbackTermToPack.changeFrame(trajectoryFrame);
         yoErrorAngularVelocity.setIncludingFrame(feedbackTermToPack);
         yoFilteredErrorAngularVelocity.update();
         yoFilteredErrorAngularVelocity.setCommandId(currentCommandId);
         feedbackTermToPack.set(yoFilteredErrorAngularVelocity);
      }
      else
      {
         yoErrorAngularVelocity.setIncludingFrame(feedbackTermToPack);
      }
      yoErrorAngularVelocity.changeFrame(trajectoryFrame);
      yoErrorAngularVelocity.setCommandId(currentCommandId);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(endEffectorFrame);

      feedbackTermToPack.get(0, spatialError);
      CommonOps_DDRM.mult(mptcGainsCalculator.getDerivativeGains(), spatialError, spatialFeedbackTerm);
      feedbackTermToPack.set(0, spatialFeedbackTerm);

      feedbackTermToPack.changeFrame(endEffectorFrame);
   }
}
