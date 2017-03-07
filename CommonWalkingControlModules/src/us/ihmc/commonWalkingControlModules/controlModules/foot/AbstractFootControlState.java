package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class AbstractFootControlState extends FinishableState<ConstraintType>
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final FootControlHelper footControlHelper;
   protected final PrivilegedConfigurationCommand straightLegsPrivilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   protected final PrivilegedConfigurationCommand bentLegsPrivilegedConfigurationCommand = new PrivilegedConfigurationCommand();

   protected final RobotSide robotSide;
   protected final RigidBody rootBody;
   protected final RigidBody pelvis;
   protected final ContactableFoot contactableFoot;

   protected final FramePoint desiredPosition = new FramePoint(worldFrame);
   protected final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredLinearAcceleration = new FrameVector(worldFrame);
   protected final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   protected final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);
   protected final SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();

   protected final HighLevelHumanoidControllerToolbox controllerToolbox;

   protected boolean attemptToStraightenLegs;

   public AbstractFootControlState(ConstraintType stateEnum, FootControlHelper footControlHelper)
   {
      super(stateEnum);

      this.footControlHelper = footControlHelper;
      this.contactableFoot = footControlHelper.getContactableFoot();

      this.controllerToolbox = footControlHelper.getHighLevelHumanoidControllerToolbox();

      this.robotSide = footControlHelper.getRobotSide();

      rootBody = controllerToolbox.getTwistCalculator().getRootBody();

      FullHumanoidRobotModel fullRobotModel = footControlHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      RigidBody foot = fullRobotModel.getFoot(robotSide);
      OneDoFJoint kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH);

      straightLegsPrivilegedConfigurationCommand.addJoint(kneeJoint, PrivilegedConfigurationOption.AT_ZERO);

      bentLegsPrivilegedConfigurationCommand.addJoint(kneeJoint, PrivilegedConfigurationOption.AT_MID_RANGE);
   }

   public abstract void doSpecificAction();

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   @Override
   public void doAction()
   {
      doSpecificAction();
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   /**
    * Determines whether or not the privileged configuration command that is utilized is at the mid range or at zero
    * Linked to {@link WalkingControllerParameters#attemptToStraightenLegs()}
    * @param attemptToStraightenLegs boolean (true = at zero, false = at mid range)
    */
   public void setAttemptToStraightenLegs(boolean attemptToStraightenLegs)
   {
      this.attemptToStraightenLegs = attemptToStraightenLegs;
   }
}
