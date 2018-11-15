package us.ihmc.quadrupedRobotics.controller.states;

import java.util.BitSet;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedJointInitializationController implements State
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullQuadrupedRobotModel fullRobotModel;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final YoBoolean forceFeedbackControlEnabled;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the {@link FullRobotModel#getOneDoFJoints()}
    * array.
    */
   private final BitSet initialized;

   public QuadrupedJointInitializationController(QuadrupedRuntimeEnvironment environment, QuadrupedControlMode controlMode, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = environment.getFullRobotModel();
      this.jointDesiredOutputList = environment.getJointDesiredOutputList();
      this.initialized = new BitSet(fullRobotModel.getOneDoFJoints().length);

      forceFeedbackControlEnabled = new YoBoolean("useForceFeedbackControl", registry);
      forceFeedbackControlEnabled.set(controlMode == QuadrupedControlMode.FORCE);

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      OneDoFJointBasics[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         if (fullRobotModel.getNameForOneDoFJoint(joint).getRole() == JointRole.LEG)
         {
            if (forceFeedbackControlEnabled.getBooleanValue())
               jointDesiredOutputList.getJointDesiredOutput(joint).setControlMode(JointDesiredControlMode.EFFORT);
            else
               jointDesiredOutputList.getJointDesiredOutput(joint).setControlMode(JointDesiredControlMode.POSITION);
         }
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      OneDoFJointBasics[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];

         // Only set a desired if the actuator has just come online.
         if (!initialized.get(i)) //FIXME && joint.isOnline())
         {
            jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredTorque(0.0);
            jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredPosition(joint.getQ());

            initialized.set(i);
         }
      }

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return allJointsInitialized();
   }

   @Override
   public void onExit()
   {
   }

   private boolean allJointsInitialized()
   {
      return initialized.cardinality() == initialized.length();
   }
}

