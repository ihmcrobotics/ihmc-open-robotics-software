package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedDoNothingController extends QuadrupedController implements QuadrupedJointInitializer
{
   private final FullRobotModel fullRobotModel;
   private final boolean initialized[];

   public QuadrupedDoNothingController(FullRobotModel fullRobotModel)
   {
      super(QuadrupedControllerState.DO_NOTHING);

      this.fullRobotModel = fullRobotModel;
      this.initialized = new boolean[fullRobotModel.getOneDoFJoints().length];
   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];

         // Only set a desired if the actuator has just come online.
         if (!initialized[i] && joint.isOnline())
         {
            joint.setqDesired(joint.getQ());
            initialized[i] = true;
         }
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joint.setUnderPositionControl(true);
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

   @Override
   public boolean allJointsInitialized()
   {
      for (boolean init : initialized)
      {
         if (!init)
         {
            return false;
         }
      }

      return true;
   }
}

