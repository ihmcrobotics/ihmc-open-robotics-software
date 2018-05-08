package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedJointIntegrator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final OneDoFJoint controllerJoint;
   private final JointDesiredOutput jointDesiredSetpoints;

   private final double controlDT;
   private final YoDouble integratedVelocity;
   private final YoDouble integratedPosition;
   private boolean resetIntegrators;

   public QuadrupedJointIntegrator(OneDoFJoint controllerJoint, JointDesiredOutput jointDesiredSetpoints, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;

      registry = new YoVariableRegistry(controllerJoint.getName() + name);

      this.controllerJoint = controllerJoint;
      this.jointDesiredSetpoints = jointDesiredSetpoints;

      integratedVelocity = new YoDouble(controllerJoint.getName() + "_IntegratedVelocity", registry);
      integratedPosition = new YoDouble(controllerJoint.getName() + "_IntegratedPosition", registry);

      this.resetIntegrators = true;

      parentRegistry.addChild(registry);
   }

   public void computeDesiredStateFromJointController()
   {
      if (jointDesiredSetpoints.pollResetIntegratorsRequest())
      {
         resetIntegrators = true;
      }

      // If no desired acceleration is available or resetIntegrators flag is set then
      // initialize integrated position and velocity to their current estimates.
      if (!jointDesiredSetpoints.hasDesiredAcceleration())
      {
         integratedVelocity.set(controllerJoint.getQd());
         integratedPosition.set(controllerJoint.getQ());
         return;
      }

      if (resetIntegrators)
      {
         integratedVelocity.set(controllerJoint.getQd());
         integratedPosition.set(controllerJoint.getQ());
         resetIntegrators = false;
      }

      // If no desired velocity is provided then integrate the desired acceleration to compute it. This integrator leaks
      // to the current joint velocity and joint position.
      if (!jointDesiredSetpoints.hasDesiredVelocity())
      {
         double kd = jointDesiredSetpoints.hasVelocityIntegrationBreakFrequency() ?
               AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointDesiredSetpoints.getVelocityIntegrationBreakFrequency(), controlDT) :
               1.0;
         integratedVelocity.add(kd * (controllerJoint.getQd() - integratedVelocity.getDoubleValue()) + jointDesiredSetpoints.getDesiredAcceleration() * controlDT);
         jointDesiredSetpoints.setDesiredVelocity(integratedVelocity.getDoubleValue());
      }
      else
      {
         integratedVelocity.set(controllerJoint.getQd());
      }

      // If no desired position is available then integrate the desired velocity to compute it.
      if (!jointDesiredSetpoints.hasDesiredPosition())
      {
         double kp = jointDesiredSetpoints.hasPositionIntegrationBreakFrequency() ?
               AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointDesiredSetpoints.getPositionIntegrationBreakFrequency(), controlDT) :
               1.0;
         integratedPosition.add(kp * (controllerJoint.getQ() - integratedPosition.getDoubleValue()) + integratedVelocity.getDoubleValue() * controlDT);
         jointDesiredSetpoints.setDesiredPosition(integratedPosition.getDoubleValue());
      }
      else
      {
         integratedPosition.set(controllerJoint.getQ());
      }
   }
}
