package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedLowLevelJointController
{
   private final double dt;
   private YoDouble integratedVelocity;
   private YoDouble integratedPosition;
   private boolean resetIntegrators;

   private final YoDouble jointErrorFeedback;
   private final YoDouble jointEffort;
   private final YoDouble jointDampingFeedback;

   public QuadrupedLowLevelJointController(String prefix, double dt, YoVariableRegistry registry)
   {
      this.dt = dt;

      integratedVelocity = new YoDouble(prefix + "_IntegratedVelocity", registry);
      integratedPosition = new YoDouble(prefix + "_IntegratedPosition", registry);
      jointErrorFeedback = new YoDouble(prefix + "_JointErrorFeedback", registry);
      jointEffort = new YoDouble(prefix + "_JointEffort", registry);
      jointDampingFeedback = new YoDouble(prefix + "_JointDampingFeedback", registry);

      this.resetIntegrators = true;
   }

   public void resetIntegrators()
   {
      resetIntegrators = true;
   }

   public void update(OneDoFJoint jointEstimates, JointDesiredOutput jointSetpoints)
   {
      computePositionAndVelocitySetpoints(jointEstimates, jointSetpoints);
      computeEffort(jointEstimates, jointSetpoints);
   }

   private void computePositionAndVelocitySetpoints(OneDoFJoint jointEstimates, JointDesiredOutput jointSetpoints)
   {
      // If no desired acceleration is available or resetIntegrators flag is set then
      // initialize integrated position and velocity to their current estimates.
      if (!jointSetpoints.hasDesiredAcceleration())
      {
         integratedVelocity.set(jointEstimates.getQd());
         integratedPosition.set(jointEstimates.getQ());
         return;
      }

      if (resetIntegrators)
      {
         integratedVelocity.set(jointEstimates.getQd());
         integratedPosition.set(jointEstimates.getQ());
         resetIntegrators = false;
      }

      // If no desired velocity is provided then integrate the desired acceleration to compute it. This integrator leaks
      // to the current joint velocity and joint position.
      if (!jointSetpoints.hasDesiredVelocity())
      {
         double kd = jointSetpoints.hasVelocityIntegrationBreakFrequency() ?
               AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSetpoints.getVelocityIntegrationBreakFrequency(), dt) :
               1.0;
         integratedVelocity.add(kd * (jointEstimates.getQd() - integratedVelocity.getDoubleValue()) + jointSetpoints.getDesiredAcceleration() * dt);
         jointSetpoints.setDesiredVelocity(integratedVelocity.getDoubleValue());
      }
      else
      {
         integratedVelocity.set(jointEstimates.getQd());
      }

      // If no desired position is available then integrate the desired velocity to compute it.
      if (!jointSetpoints.hasDesiredPosition())
      {
         double kp = jointSetpoints.hasPositionIntegrationBreakFrequency() ?
               AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointSetpoints.getPositionIntegrationBreakFrequency(), dt) :
               1.0;
         integratedPosition.add(kp * (jointEstimates.getQ() - integratedPosition.getDoubleValue()) + integratedVelocity.getDoubleValue() * dt);
         jointSetpoints.setDesiredPosition(integratedPosition.getDoubleValue());
      }
      else
      {
         integratedPosition.set(jointEstimates.getQ());
      }
   }

   private void computeEffort(OneDoFJoint jointEstimates, JointDesiredOutput jointSetpoints)
   {
      double jointStiffness = jointSetpoints.hasStiffness() ? jointSetpoints.getStiffness() : 0.0;
      double jointDamping = jointSetpoints.hasDamping() ? jointSetpoints.getDamping() : 0.0;

      double desiredEffort = jointSetpoints.hasDesiredTorque() ? jointSetpoints.getDesiredTorque() : 0.0;

      double desiredVelocity = jointSetpoints.hasDesiredVelocity() ? jointSetpoints.getDesiredVelocity() : 0.0;
      if (jointSetpoints.hasVelocityScaling())
         desiredVelocity *= jointSetpoints.getVelocityScaling();

      double jointDampingFeedback = jointDamping * (desiredVelocity - jointEstimates.getQd());
      this.jointDampingFeedback.set(jointDampingFeedback);
      desiredEffort += jointDampingFeedback;

      double desiredPosition = jointSetpoints.hasDesiredPosition() ? jointSetpoints.getDesiredPosition() : jointEstimates.getQ();
      double jointErrorFeedback = jointStiffness * (desiredPosition - jointEstimates.getQ());
      this.jointErrorFeedback.set(jointErrorFeedback);
      desiredEffort += jointErrorFeedback;

      this.jointEffort.set(desiredEffort);

      jointSetpoints.setDesiredTorque(desiredEffort);
   }
}
