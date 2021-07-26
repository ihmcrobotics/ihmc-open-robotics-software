package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlHelper;
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

public interface JumpingFootControlState extends State
{
   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract SpatialFeedbackControlCommand getFeedbackControlCommand();


   @Override
   default boolean isDone(double timeInState)
   {
      return true;
   }
}
