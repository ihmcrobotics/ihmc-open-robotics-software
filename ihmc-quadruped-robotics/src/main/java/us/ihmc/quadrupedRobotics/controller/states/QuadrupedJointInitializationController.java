package us.ihmc.quadrupedRobotics.controller.states;

import java.util.BitSet;

import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedJointInitializationController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullQuadrupedRobotModel fullRobotModel;
   private final JointDesiredOutputList jointDesiredOutputList;

   private final QuadrupedControllerToolbox controllerToolbox;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the {@link FullRobotModel#getOneDoFJoints()}
    * array.
    */
   private final BitSet initialized;

   public QuadrupedJointInitializationController(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.fullRobotModel = controllerToolbox.getFullRobotModel();
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();
      this.initialized = new BitSet(fullRobotModel.getOneDoFJoints().length);

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         if (fullRobotModel.getNameForOneDoFJoint(joint).getRole() == JointRole.LEG)
         {
            if (controllerToolbox.isPositionControlled())
               jointDesiredOutputList.getJointDesiredOutput(joint).setControlMode(JointDesiredControlMode.POSITION);
            else
               jointDesiredOutputList.getJointDesiredOutput(joint).setControlMode(JointDesiredControlMode.EFFORT);

         }
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         // Only set a desired if the actuator has just come online.
         if (!initialized.get(i) && joint.isOnline())
         {
            jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredTorque(0.0);
            jointDesiredOutputList.getJointDesiredOutput(joint).setDesiredPosition(joint.getQ());

            initialized.set(i);
         }
      }

   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return allJointsInitialized() ? ControllerEvent.DONE : null;
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

