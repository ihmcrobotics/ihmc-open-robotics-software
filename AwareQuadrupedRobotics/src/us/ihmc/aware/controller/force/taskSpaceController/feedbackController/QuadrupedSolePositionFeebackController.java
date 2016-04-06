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
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Vector3d;

public class QuadrupedSolePositionFeebackController implements QuadrupedTaskSpaceFeedbackController
{
   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame soleFrame;
   private final EuclideanPositionController solePositionController;
   private final YoEuclideanPositionGains solePositionFeedbackGains;

   public QuadrupedSolePositionFeebackController(RobotQuadrant robotQuadrant, ReferenceFrame soleFrame, double controlDT, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.soleFrame = soleFrame;
      solePositionController = new EuclideanPositionController(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SolePosition", soleFrame, controlDT, registry);
      solePositionFeedbackGains = new YoEuclideanPositionGains(robotQuadrant.getCamelCaseNameForStartOfExpression() + "SolePosition", registry);
   }

   public YoEuclideanPositionGains getGains()
   {
      return solePositionFeedbackGains;
   }

   @Override public void reset()
   {
      solePositionController.reset();
   }

   @Override public void computeFeedback(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceSetpoints setpoints, QuadrupedTaskSpaceCommands commands)
   {
      FrameVector soleForceCommand = commands.getSoleForce(robotQuadrant);
      FramePoint solePositionSetpoint = setpoints.getSolePosition(robotQuadrant);
      FrameVector soleLinearVelocitySetpoint = setpoints.getSoleLinearVelocity(robotQuadrant);
      FrameVector soleLinearVelocityEstimate = estimates.getSoleLinearVelocity(robotQuadrant);
      FrameVector soleForceFeedforwardSetpoint = setpoints.getSoleForceFeedforward(robotQuadrant);

      // compute sole force
      soleForceCommand.setToZero(soleFrame);
      solePositionSetpoint.changeFrame(soleFrame);
      soleLinearVelocitySetpoint.changeFrame(soleFrame);
      soleLinearVelocityEstimate.changeFrame(soleFrame);
      soleForceFeedforwardSetpoint.changeFrame(soleFrame);
      solePositionController.setGains(solePositionFeedbackGains);
      solePositionController.compute(soleForceCommand, solePositionSetpoint, soleLinearVelocitySetpoint, soleLinearVelocityEstimate, soleForceFeedforwardSetpoint);
   }

   @Override public void registerGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {

   }
}