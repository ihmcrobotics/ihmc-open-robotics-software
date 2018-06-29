package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final YoPlaneContactState contactState;
   private final ReferenceFrame soleFrame;

   private final FootSwitchInterface footSwitch;

   private final SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
   private boolean footIsVerifiedAsLoaded = false;

   private final DoubleParameter minimumTimeInSupportState;

   private final YoFramePoint3D groundPlanePosition;
   private final YoFramePoint3D upcomingGroundPlanePosition;

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.groundPlanePosition = controllerToolbox.getGroundPlanePositions().get(robotQuadrant);
      this.upcomingGroundPlanePosition = controllerToolbox.getUpcomingGroundPlanePositions().get(robotQuadrant);
      this.contactState = controllerToolbox.getFootContactState(robotQuadrant);
      this.soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);

      minimumTimeInSupportState = new DoubleParameter(robotQuadrant.getShortName() + "TimeInSupportState", registry, 0.05);

      footSwitch = controllerToolbox.getRuntimeEnvironment().getFootSwitches().get(robotQuadrant);

      spatialVelocityCommand.set(controllerToolbox.getFullRobotModel().getElevator(), controllerToolbox.getFullRobotModel().getFoot(robotQuadrant));
      spatialVelocityCommand.setSelectionMatrixForLinearControl();
      spatialVelocityCommand.setWeight(10.0);
   }

   @Override
   public void onEntry()
   {
      contactState.setFullyConstrained();
      contactState.setContactNormalVector(footNormalContactVector);

      if (waypointCallback != null)
         waypointCallback.isDoneMoving(robotQuadrant, true);

      footIsVerifiedAsLoaded = false;

   }


   private final FramePoint3D tempPoint = new FramePoint3D();
   @Override
   public void doAction(double timeInState)
   {
      if (footSwitch.hasFootHitGround())
      {
         if (!footIsVerifiedAsLoaded && timeInState > minimumTimeInSupportState.getValue())
         {
            footIsVerifiedAsLoaded = true;

            tempPoint.setToZero(soleFrame);
            groundPlanePosition.setMatchingFrame(tempPoint);
            upcomingGroundPlanePosition.setMatchingFrame(tempPoint);
         }
      }
      spatialVelocityCommand.setSpatialVelocityToZero(soleFrame);
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
      footIsVerifiedAsLoaded = false;
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public InverseKinematicsCommand<?> getInverseKinematicsCommand()
   {
      return null;
//      return spatialVelocityCommand;
   }


   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}
