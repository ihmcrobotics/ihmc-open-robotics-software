package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

public class QuadrupedStandReadyController extends QuadrupedController
{
   private final SDFFullRobotModel fullRobotModel;

   public QuadrupedStandReadyController(SDFFullRobotModel fullRobotModel)
   {
      super(QuadrupedControllerState.STAND_READY);

      this.fullRobotModel = fullRobotModel;
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for(OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joint.setUnderPositionControl(true);
         
         // Set the desired to the initial, then never touch it again.
         joint.setqDesired(joint.getQ());
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.STANDING;
   }
}
