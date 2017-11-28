package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class AbstractFootControlState extends FinishableState<ConstraintType>
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final FootControlHelper footControlHelper;

   protected final RobotSide robotSide;
   protected final RigidBody rootBody;
   protected final RigidBody pelvis;
   protected final ContactableFoot contactableFoot;

   protected final FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
   protected final FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame);
   protected final FrameVector3D desiredLinearAcceleration = new FrameVector3D(worldFrame);
   protected final FrameQuaternion desiredOrientation = new FrameQuaternion(worldFrame);
   protected final FrameVector3D desiredAngularVelocity = new FrameVector3D(worldFrame);
   protected final FrameVector3D desiredAngularAcceleration = new FrameVector3D(worldFrame);
   protected final SpatialAccelerationVector footAcceleration = new SpatialAccelerationVector();

   protected final HighLevelHumanoidControllerToolbox controllerToolbox;

   public AbstractFootControlState(ConstraintType stateEnum, FootControlHelper footControlHelper)
   {
      super(stateEnum);

      this.footControlHelper = footControlHelper;
      this.contactableFoot = footControlHelper.getContactableFoot();

      this.controllerToolbox = footControlHelper.getHighLevelHumanoidControllerToolbox();

      this.robotSide = footControlHelper.getRobotSide();
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      rootBody = fullRobotModel.getElevator();
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
}
