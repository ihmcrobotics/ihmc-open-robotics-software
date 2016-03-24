package us.ihmc.aware.controller.force.taskSpaceController.feedbackController;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceSetpoints;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class QuadrupedComPositionFeedbackController implements QuadrupedTaskSpaceFeedbackController
{
   private final ReferenceFrame comZUpFrame;
   private final EuclideanPositionController comPositionController;
   private final YoEuclideanPositionGains comPositionFeedbackGains;

   public QuadrupedComPositionFeedbackController(ReferenceFrame comZUpFrame, double controlDT, YoVariableRegistry registry)
   {
      this.comZUpFrame = comZUpFrame;
      comPositionController = new EuclideanPositionController("comPosition", comZUpFrame, controlDT, registry);
      comPositionFeedbackGains = new YoEuclideanPositionGains("comPosition", registry);
   }

   public YoEuclideanPositionGains getGains()
   {
      return comPositionFeedbackGains;
   }

   @Override public void reset()
   {
      comPositionController.reset();
   }

   @Override public void computeFeedback(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceSetpoints setpoints, QuadrupedTaskSpaceCommands commands)
   {
      FrameVector comForceCommand = commands.getComForce();
      FramePoint comPositionSetpoint = setpoints.getComPosition();
      FrameVector comVelocitySetpoint = setpoints.getComVelocity();
      FrameVector comVelocityEstimate = estimates.getComVelocity();
      FrameVector comForceFeedforwardSetpoint = setpoints.getComForceFeedforward();

      // compute com force
      comForceCommand.setToZero(comZUpFrame);
      comPositionSetpoint.changeFrame(comZUpFrame);
      comVelocitySetpoint.changeFrame(comZUpFrame);
      comVelocityEstimate.changeFrame(comZUpFrame);
      comForceFeedforwardSetpoint.changeFrame(comZUpFrame);
      comPositionController.setGains(comPositionFeedbackGains);
      comPositionController.compute(comForceCommand, comPositionSetpoint, comVelocitySetpoint, comVelocityEstimate, comForceFeedforwardSetpoint);
   }

   @Override public void enableVisualization(boolean enable)
   {
   }
}
