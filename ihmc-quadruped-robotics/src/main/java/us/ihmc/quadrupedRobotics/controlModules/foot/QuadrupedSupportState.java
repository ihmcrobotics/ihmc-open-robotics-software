package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final YoPlaneContactState contactState;

   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;
   private boolean triggerSwing;

   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, YoPlaneContactState contactState, YoBoolean stepCommandIsValid, YoDouble timestamp,
                                YoQuadrupedTimedStep stepCommand)
   {
      this.robotQuadrant = robotQuadrant;
      this.contactState = contactState;
      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = timestamp;
      this.currentStepCommand = stepCommand;
   }

   @Override
   public void onEntry()
   {
      contactState.setFullyConstrained();
      contactState.setContactNormalVector(footNormalContactVector);
      triggerSwing = false;
   }

   @Override
   public void doAction(double timeInState)
   {
      // trigger swing phase
      if (stepCommandIsValid.getBooleanValue() && currentStepCommand.getTimeInterval().intervalContains(timestamp.getDoubleValue()))
      {
         if (stepTransitionCallback != null)
         {
            stepTransitionCallback.onLiftOff(robotQuadrant);
         }
         triggerSwing = true;
      }
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return triggerSwing ? QuadrupedFootControlModule.FootEvent.TIMEOUT : null;
   }

   @Override
   public void onExit()
   {
      triggerSwing = false;
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
}
