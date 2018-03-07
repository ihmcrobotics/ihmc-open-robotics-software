package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.lang.ref.Reference;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame soleFrame;
   private final YoPlaneContactState contactState;

   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;

   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);


   public QuadrupedSupportState(RobotQuadrant robotQuadrant, ReferenceFrame soleFrame, YoPlaneContactState contactState,
                                YoBoolean stepCommandIsValid, YoDouble timestamp, YoQuadrupedTimedStep stepCommand)
   {
      this.robotQuadrant = robotQuadrant;
      this.soleFrame = soleFrame;
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
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      // trigger swing phase
      if (stepCommandIsValid.getBooleanValue() && currentStepCommand.getTimeInterval().intervalContains(timestamp.getDoubleValue()))
      {
         if (stepTransitionCallback != null)
         {
            stepTransitionCallback.onLiftOff(robotQuadrant);
         }

         return QuadrupedFootControlModule.FootEvent.TIMEOUT;
      }

      return null;
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }
}
