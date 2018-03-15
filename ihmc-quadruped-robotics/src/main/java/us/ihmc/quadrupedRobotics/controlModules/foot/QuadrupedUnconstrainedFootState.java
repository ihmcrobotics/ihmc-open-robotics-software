package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public abstract class QuadrupedUnconstrainedFootState extends QuadrupedFootState
{
   protected final QuadrupedSolePositionController solePositionController;
   protected final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;

   protected final FrameVector3D initialSoleForces = new FrameVector3D();

   protected final FrameVector3D desiredLinearAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());

   protected final VirtualForceCommand virtualForceCommand = new VirtualForceCommand();
   //private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();


   //private final YoFrameVector currentLinearWeight;

   public QuadrupedUnconstrainedFootState(String namePrefix, RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox,
                                          QuadrupedSolePositionController solePositionController, YoVariableRegistry registry)
   {
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      FullQuadrupedRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      virtualForceCommand.set(fullRobotModel.getBody(), fullRobotModel.getFoot(robotQuadrant));

      /*
      ReferenceFrame gainsFrame = controllerToolbox.getReferenceFrames().getBodyZUpFrame();
      FramePose3D controlFramePose = new FramePose3D(soleFrame);
      controlFramePose.changeFrame(footBody.getBodyFixedFrame());
      spatialFeedbackControlCommand.set(fullRobotModel.getBody(), footBody);
      spatialFeedbackControlCommand.setPrimaryBase(fullRobotModel.getBody());
      spatialFeedbackControlCommand.setGainsFrames(null, gainsFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      */


      /*
      currentLinearWeight = new YoFrameVector(namePrefix + "CurrentLinearWeight", worldFrame, registry);
      currentLinearWeight.set(30.0, 30.0, 30.0);
      */
   }

   protected void doControl()
   {
      /*
      spatialFeedbackControlCommand.set(solePositionControllerSetpoints.getSolePosition(), solePositionControllerSetpoints.getSoleLinearVelocity(), desiredLinearAcceleration);
      spatialFeedbackControlCommand.setLinearWeightsForSolver(currentLinearWeight);
      spatialFeedbackControlCommand.setPositionGains(solePositionController.getGains());
      */
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }


   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return virtualForceCommand;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }
}
