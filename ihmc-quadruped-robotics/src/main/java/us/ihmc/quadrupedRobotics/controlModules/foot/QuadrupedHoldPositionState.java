package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedHoldPositionState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // YoVariables
   protected final QuadrupedControllerToolbox controllerToolbox;
   protected final RobotQuadrant robotQuadrant;

   private final YoDouble timestamp;
   private double initialTime;

   private final ReferenceFrame soleFrame;
   private final QuadrupedFootControlModuleParameters parameters;

   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootVelocity = new FrameVector3D();

   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   public QuadrupedHoldPositionState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      parameters = controllerToolbox.getFootControlModuleParameters();
      timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.getFootContactState(robotQuadrant).clear();

      initialTime = timestamp.getDoubleValue();

      desiredFootPosition.setToZero(soleFrame);
      desiredFootPosition.changeFrame(worldFrame);

   }

   @Override
   public void doAction(double timeInState)
   {
      desiredFootVelocity.setToZero();

      feedbackControlCommand.set(desiredFootPosition, desiredFootVelocity);
      feedbackControlCommand.setGains(parameters.getSolePositionGains());
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}
