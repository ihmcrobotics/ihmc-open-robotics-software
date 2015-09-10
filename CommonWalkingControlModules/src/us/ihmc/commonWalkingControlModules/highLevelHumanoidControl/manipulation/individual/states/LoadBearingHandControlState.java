package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;


/**
 * @author twan
 *         Date: 5/30/13
 */
public class LoadBearingHandControlState extends TaskspaceHandControlState
{
   protected final DoubleYoVariable coefficientOfFriction;
   protected final SpatialAccelerationVector handAcceleration;

   private final FrameVector contactNormal;

   private final ContactablePlaneBody handPalm;
   private final ReferenceFrame handControlFrame;
   private final ReferenceFrame handFrame;
   private final ReferenceFrame elevatorFrame;

   private final FramePose desiredPose = new FramePose();
   private final Twist desiredTwist = new Twist();
   private final SpatialAccelerationVector desiredAcceleration = new SpatialAccelerationVector();

   private RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule;

   public LoadBearingHandControlState(String namePrefix, HandControlState stateEnum, RobotSide robotSide, MomentumBasedController momentumBasedController,
         RigidBody elevator, RigidBody endEffector, int jacobianId, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, elevator, endEffector, parentRegistry);
      
      coefficientOfFriction = new DoubleYoVariable(name + "CoefficientOfFriction", registry);
      handAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(), endEffector.getBodyFixedFrame());

      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      elevatorFrame = fullRobotModel.getElevatorFrame();
      handFrame = fullRobotModel.getHand(robotSide).getBodyFixedFrame();
      
      SideDependentList<ContactablePlaneBody> contactableHands = momentumBasedController.getContactableHands();
      if (contactableHands != null)
      {
         handPalm = contactableHands.get(robotSide);
         handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         contactNormal = new FrameVector();
         contactNormal.setToNaN();
         momentumBasedController.setPlaneContactStateFree(handPalm);
      }
      else
      {
         handPalm = null;
         handControlFrame = null;
         contactNormal = null;
      }
   }

   @Override
   public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule)
   {
      this.handRigidBodySpatialAccelerationControlModule = handRigidBodySpatialAccelerationControlModule;
   }

   @Override
   public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
   {
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalVector(FrameVector normal)
   {
      double length = normal.length();
      if (length < 1e-7)
      {
         contactNormal.setToNaN();
      }
      else
      {
         contactNormal.setIncludingFrame(normal);
      }
   }

   public void getContactNormalVector(FrameVector normalToPack)
   {
      normalToPack.setIncludingFrame(contactNormal);
   }

   @Override
   public void doAction()
   {
      desiredTwist.setToZero(handControlFrame, elevatorFrame, handControlFrame);
      desiredAcceleration.setToZero(handControlFrame, elevatorFrame, handControlFrame);
      handRigidBodySpatialAccelerationControlModule.doPositionControl(desiredPose, desiredTwist, desiredAcceleration, getBase());
      handRigidBodySpatialAccelerationControlModule.packAcceleration(handAcceleration);
      handAcceleration.changeBodyFrameNoRelativeAcceleration(handFrame);
      handAcceleration.changeFrameNoRelativeMotion(handFrame);

      submitDesiredAcceleration(handAcceleration);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (handPalm == null)
      {
         System.out.println("No YoPlaneContactState for the hands in the MomentumBasedController, leaving the hand load bearing state.");

         return;
      }

      if (contactNormal.containsNaN())
         contactNormal.setIncludingFrame(handPalm.getSoleFrame(), 0.0, 0.0, 1.0);

      contactNormal.changeFrame(ReferenceFrame.getWorldFrame());
      momentumBasedController.setPlaneContactStateFullyConstrained(handPalm, coefficientOfFriction.getDoubleValue(), contactNormal);
      desiredPose.setToZero(handControlFrame);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (handPalm == null)
         return;

      momentumBasedController.setPlaneContactStateFree(handPalm);
      contactNormal.setToNaN();
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}
