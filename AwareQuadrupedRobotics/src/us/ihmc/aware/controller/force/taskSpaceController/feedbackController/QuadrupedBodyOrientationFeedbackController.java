package us.ihmc.aware.controller.force.taskSpaceController.feedbackController;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceSetpoints;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.YoAxisAngleOrientationGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class QuadrupedBodyOrientationFeedbackController implements QuadrupedTaskSpaceFeedbackController
{
   private final ReferenceFrame bodyFrame;
   private final AxisAngleOrientationController bodyOrientationController;
   private final YoAxisAngleOrientationGains bodyOrientationFeedbackGains;

   public QuadrupedBodyOrientationFeedbackController(ReferenceFrame bodyFrame, double controlDT, YoVariableRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, controlDT, registry);
      bodyOrientationFeedbackGains = new YoAxisAngleOrientationGains("bodyOrientation", registry);
   }

   public YoAxisAngleOrientationGains getGains()
   {
      return bodyOrientationFeedbackGains;
   }

   @Override public void reset()
   {
      bodyOrientationController.reset();
   }

   @Override public void computeFeedback(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceSetpoints setpoints, QuadrupedTaskSpaceCommands commands)
   {
      FrameVector comTorqueCommand = commands.getComTorque();
      FrameOrientation bodyOrientationSetpoint = setpoints.getBodyOrientation();
      FrameVector bodyAngularVelocitySetpoint = setpoints.getBodyAngularVelocity();
      FrameVector bodyAngularVelocityEstimate = estimates.getBodyAngularVelocity();
      FrameVector comTorqueFeedforwardSetpoint = setpoints.getComTorqueFeedforward();

      // compute body torque
      comTorqueCommand.setToZero(bodyFrame);
      bodyOrientationSetpoint.changeFrame(bodyFrame);
      bodyAngularVelocitySetpoint.changeFrame(bodyFrame);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);
      comTorqueFeedforwardSetpoint.changeFrame(bodyFrame);
      bodyOrientationController.setGains(bodyOrientationFeedbackGains);
      bodyOrientationController.compute(comTorqueCommand, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, comTorqueFeedforwardSetpoint);
   }

   @Override public void enableVisualization(boolean enable)
   {
   }
}