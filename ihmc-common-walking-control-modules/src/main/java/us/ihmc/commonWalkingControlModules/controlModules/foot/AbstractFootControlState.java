package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;

public abstract class AbstractFootControlState implements State
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final FootControlHelper footControlHelper;

   protected final RobotSide robotSide;
   protected final RigidBodyBasics rootBody;
   protected final RigidBodyBasics pelvis;
   protected final ContactableFoot contactableFoot;

   protected final FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
   protected final FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame);
   protected final FrameVector3D desiredLinearAcceleration = new FrameVector3D(worldFrame);
   protected final FrameQuaternion desiredOrientation = new FrameQuaternion(worldFrame);
   protected final FrameVector3D desiredAngularVelocity = new FrameVector3D(worldFrame);
   protected final FrameVector3D desiredAngularAcceleration = new FrameVector3D(worldFrame);
   protected final SpatialAcceleration footAcceleration = new SpatialAcceleration();

   protected final HighLevelHumanoidControllerToolbox controllerToolbox;

   public AbstractFootControlState(FootControlHelper footControlHelper)
   {
      this.footControlHelper = footControlHelper;
      contactableFoot = footControlHelper.getContactableFoot();

      controllerToolbox = footControlHelper.getHighLevelHumanoidControllerToolbox();

      robotSide = footControlHelper.getRobotSide();
      FullHumanoidRobotModel fullRobotModel = footControlHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      rootBody = fullRobotModel.getElevator();
   }

   public abstract void doSpecificAction(double timeInState);

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract SpatialFeedbackControlCommand getFeedbackControlCommand();

   @Override
   public void doAction(double timeInState)
   {
      doSpecificAction(timeInState);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return true;
   }

   public Object pollStatusToReport()
   {
      return null;
   }
}
