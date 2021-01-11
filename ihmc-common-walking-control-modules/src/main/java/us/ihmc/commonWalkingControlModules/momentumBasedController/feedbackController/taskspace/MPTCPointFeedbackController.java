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

public class MPTCPointFeedbackController extends PointFeedbackController
{
   private final MPTCGainsCalculator mptcGainsCalculator;

   public MPTCPointFeedbackController(RigidBodyBasics endEffector, int controllerIndex, WholeBodyControlCoreToolbox ccToolbox,
                                      FeedbackControllerToolbox fbToolbox, YoRegistry parentRegistry)
   {
      super(endEffector, controllerIndex, ccToolbox, fbToolbox, parentRegistry);
      mptcGainsCalculator = new MPTCGainsCalculator(MultiBodySystemTools.getRootBody(endEffector));
   }

   @Override
   public void computeInverseDynamics()
   {
      mptcGainsCalculator.computeMPTCGains(null, gains, endEffector, controlFrame);
      super.computeInverseDynamics();
   }

   private final DMatrixRMaj spatialError = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj spatialFeedbackTerm = new DMatrixRMaj(6, 1);

   @Override
   protected void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      yoCurrentPosition.setToZero(controlFrame);
      yoCurrentPosition.changeFrame(trajectoryFrame);
      yoCurrentPosition.setCommandId(currentCommandId);

      desiredPosition.setIncludingFrame(yoDesiredPosition);
      desiredPosition.changeFrame(controlFrame);

      feedbackTermToPack.setIncludingFrame(desiredPosition);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumProportionalError());

      yoErrorPosition.setIncludingFrame(feedbackTermToPack);
      yoErrorPosition.changeFrame(trajectoryFrame);
      yoErrorPosition.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      feedbackTermToPack.get(3, spatialError);
      CommonOps_DDRM.mult(mptcGainsCalculator.getProportionalGains(), spatialError, spatialFeedbackTerm);

      feedbackTermToPack.set(3, spatialFeedbackTerm);

      feedbackTermToPack.changeFrame(controlFrame);
   }

   @Override
   protected void computeDerivativeTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      yoCurrentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());
      yoCurrentLinearVelocity.changeFrame(trajectoryFrame);
      yoCurrentLinearVelocity.setCommandId(currentCommandId);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredLinearVelocity, yoCurrentLinearVelocity);
      feedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumDerivativeError());

      if (yoFilteredErrorLinearVelocity != null)
      {
         // If the trajectory frame changed reset the filter.
         if (yoFilteredErrorLinearVelocity.getReferenceFrame() != trajectoryFrame)
         {
            yoFilteredErrorLinearVelocity.setReferenceFrame(trajectoryFrame);
            yoFilteredErrorLinearVelocity.reset();
         }
         feedbackTermToPack.changeFrame(trajectoryFrame);
         yoErrorLinearVelocity.setIncludingFrame(feedbackTermToPack);
         yoFilteredErrorLinearVelocity.update();
         yoFilteredErrorLinearVelocity.setCommandId(currentCommandId);
         feedbackTermToPack.set(yoFilteredErrorLinearVelocity);
      }
      else
      {
         yoErrorLinearVelocity.setIncludingFrame(feedbackTermToPack);
      }
      yoErrorLinearVelocity.changeFrame(trajectoryFrame);
      yoErrorLinearVelocity.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      feedbackTermToPack.get(3, spatialError);
      CommonOps_DDRM.mult(mptcGainsCalculator.getDerivativeGains(), spatialError, spatialFeedbackTerm);

      feedbackTermToPack.set(3, spatialFeedbackTerm);

      feedbackTermToPack.changeFrame(controlFrame);
   }
}
