package us.ihmc.quadrupedRobotics.controller.position.states;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * This controller sets desired joint angles to their actual values when the joint comes online.
 */
public class QuadrupedPositionJointInitializationController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedPositionJointInitializationController.class.getSimpleName());
   private final FullRobotModel fullRobotModel;

   /**
    * A map specifying which joints have been come online and had their desired positions set. Indices align with the {@link FullRobotModel#getOneDoFJoints()}
    * array.
    */
   private final YoBoolean[] initialized;

   public QuadrupedPositionJointInitializationController(QuadrupedRuntimeEnvironment environment)
   {
      this.fullRobotModel = environment.getFullRobotModel();

      this.initialized = new YoBoolean[fullRobotModel.getOneDoFJoints().length];
      for (int i = 0; i < initialized.length; i++)
      {
         initialized[i] = new YoBoolean(fullRobotModel.getOneDoFJoints()[i].getName() + "Initialized", registry);
      }

      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         joint.setUnderPositionControl(true);
      }

      for (int i = 0; i < initialized.length; i++)
      {
         initialized[i].set(false);
      }
   }

   @Override
   public ControllerEvent process()
   {
      OneDoFJoint[] joints = fullRobotModel.getOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];

         // Only set a desired if the actuator has just come online or if it is still offline (just in case).
         if (!joint.isEnabled())
         {
            joint.setqDesired(joint.getQ());
            initialized[i].set(false);
         }
         else if (!initialized[i].getBooleanValue())
         {
            joint.setqDesired(joint.getQ());
            initialized[i].set(true);
         }
      }

      return allJointsInitialized() ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
   }

   private boolean allJointsInitialized()
   {
      for (int i = 0; i < initialized.length; i++)
      {
         if (!initialized[i].getBooleanValue())
         {
            return false;
         }
      }

      return true;
   }
}

