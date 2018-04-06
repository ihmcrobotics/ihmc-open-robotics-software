package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedJoint
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final LowLevelState desiredActuatorState;
   private final OneDoFJoint controllerJoint;
   private final JointDesiredOutputReadOnly jointDesiredSetpoints;

   private final QuadrupedLowLevelJointController jointController;

   public QuadrupedJoint(LowLevelState desiredActuatorState, OneDoFJoint controllerJoint, JointDesiredOutputReadOnly jointDesiredSetpoints,
                         double controlDT, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(controllerJoint.getName() + name);

      this.controllerJoint = controllerJoint;
      this.desiredActuatorState = desiredActuatorState;
      this.jointDesiredSetpoints = jointDesiredSetpoints;
      this.jointController = new QuadrupedLowLevelJointController(controllerJoint.getName(), controlDT, registry);

      parentRegistry.addChild(registry);
   }

   public void computeDesiredStateFromJointController()
   {
      if (jointDesiredSetpoints.pollResetIntegratorsRequest())
      {
         jointController.resetIntegrators();
      }
      jointController.update(controllerJoint, jointDesiredSetpoints, desiredActuatorState);
   }

   public LowLevelActuatorMode getDesiredActuatorMode()
   {
      if (jointDesiredSetpoints.hasControlMode())
      {
         switch (jointDesiredSetpoints.getControlMode())
         {
         case POSITION:
            return LowLevelActuatorMode.POSITION;
         case VELOCITY:
            return LowLevelActuatorMode.VELOCITY;
         case EFFORT:
            return LowLevelActuatorMode.EFFORT;
         case DISABLED:
            return LowLevelActuatorMode.DISABLED;
         default:
            throw new RuntimeException("Control mode " + jointDesiredSetpoints.getControlMode() + " not implemented.");
         }
      }
      return LowLevelActuatorMode.DISABLED;
   }
}
