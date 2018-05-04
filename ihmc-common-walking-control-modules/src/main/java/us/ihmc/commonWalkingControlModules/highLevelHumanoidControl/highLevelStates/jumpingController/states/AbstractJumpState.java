package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commonWalkingControlModules.controlModules.flight.JumpMessageHandler;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.FinishableState;

public abstract class AbstractJumpState extends FinishableState<JumpStateEnum>
{
   private final ReferenceFrame plannerFrame;
   private final WholeBodyMotionPlanner motionPlanner;
   private final JumpMessageHandler messageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected FramePoint3D initialPositionInState = new FramePoint3D();
   protected FrameVector3D initialVelocityInState = new FrameVector3D();
   protected FrameVector3D initialGroundReactionForceInState = new FrameVector3D();
   protected FrameVector3D initialOrientationInState = new FrameVector3D();
   protected FrameVector3D initialAngularVelocityInState = new FrameVector3D();
   protected FrameVector3D initialTorqueInState = new FrameVector3D();
   protected FramePoint3D finalPosition = new FramePoint3D();
   protected FrameVector3D finalVelocity = new FrameVector3D();
   protected FrameVector3D finalGroundReactionForce = new FrameVector3D();
   protected FrameVector3D finalOrientation = new FrameVector3D();
   protected FrameVector3D finalAngularVelocity = new FrameVector3D();
   protected FrameVector3D finalTorque = new FrameVector3D();
   protected FrameQuaternion tempQuaternion = new FrameQuaternion();

   private final Wrench tempWrench = new Wrench();

   public AbstractJumpState(JumpStateEnum stateEnum, WholeBodyMotionPlanner motionPlanner, JumpMessageHandler messageHandler,
                            HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super(stateEnum);
      this.motionPlanner = motionPlanner;
      this.plannerFrame = motionPlanner.getPlanningFrame();
      this.finalPosition.setIncludingFrame(plannerFrame, 0.0, 0.0, 0.437);
      this.messageHandler = messageHandler;
      this.controllerToolbox = controllerToolbox;
   }

   public abstract void doStateSpecificTransitionIntoAction();

   @Override
   public void doTransitionIntoAction()
   {
      PrintTools.debug("Transitioning to " + getStateEnum());
      motionPlanner.reset();
      controllerToolbox.getCenterOfMassPosition(initialPositionInState);
      controllerToolbox.getCenterOfMassVelocity(initialVelocityInState);
      SideDependentList<FootSwitchInterface> footSwitches = controllerToolbox.getFootSwitches();
      initialGroundReactionForceInState.setToZero(plannerFrame);
      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         footSwitch.computeAndPackFootWrench(tempWrench);
         tempWrench.changeFrame(plannerFrame);
         initialGroundReactionForceInState.add(tempWrench.getLinearPart());
      }
      controllerToolbox.getPelvisZUpFrame().getTransformToDesiredFrame(plannerFrame).getRotationEuler(initialOrientationInState);
      initialOrientationInState.setX(0.0);
      initialOrientationInState.setY(0.0);
      initialAngularVelocityInState.setToZero(plannerFrame);
      tempQuaternion.setEuler(initialOrientationInState);
      messageHandler.createJumpSequenceForTesting(initialPositionInState, tempQuaternion, getStateEnum());
      List<ContactState> contactStateList = messageHandler.getContactStateList();
      if (getStateEnum() == JumpStateEnum.LANDING)
         initialGroundReactionForceInState.setToZero();
      motionPlanner.setInitialState(initialPositionInState, initialVelocityInState, initialGroundReactionForceInState, initialOrientationInState, initialAngularVelocityInState, initialTorqueInState);
      motionPlanner.getNominalState(finalVelocity, finalGroundReactionForce, finalAngularVelocity, finalTorque);
      finalPosition.setX(initialPositionInState.getX());
      finalPosition.setY(initialPositionInState.getY());
      motionPlanner.setFinalState(finalPosition, finalVelocity, finalGroundReactionForce, finalOrientation, finalAngularVelocity, finalTorque);
      motionPlanner.processContactStatesAndGenerateMotionNodesForPlanning(contactStateList);
      motionPlanner.computeMotionPlan();
      doStateSpecificTransitionIntoAction();
   }
}
