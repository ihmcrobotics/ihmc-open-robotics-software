package us.ihmc.quadrupedRobotics.output;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedJointController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;

   private final OneDoFJointBasics controllerJoint;
   private final JointDesiredOutputBasics jointDesiredSetpoints;

   private final YoDouble jointErrorFeedback;
   private final YoDouble jointEffort;
   private final YoDouble jointDampingFeedback;

   public QuadrupedJointController(OneDoFJointBasics controllerJoint, JointDesiredOutputBasics jointDesiredSetpoints, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(controllerJoint.getName() + name);

      this.controllerJoint = controllerJoint;
      this.jointDesiredSetpoints = jointDesiredSetpoints;

      String prefix = controllerJoint.getName();

      jointErrorFeedback = new YoDouble(prefix + "_JointErrorFeedback", registry);
      jointEffort = new YoDouble(prefix + "_JointEffort", registry);
      jointDampingFeedback = new YoDouble(prefix + "_JointDampingFeedback", registry);

      parentRegistry.addChild(registry);
   }

   public void computeDesiredStateFromJointController()
   {
      double jointStiffness = jointDesiredSetpoints.hasStiffness() ? jointDesiredSetpoints.getStiffness() : 0.0;
      double jointDamping = jointDesiredSetpoints.hasDamping() ? jointDesiredSetpoints.getDamping() : 0.0;

      double desiredEffort = jointDesiredSetpoints.hasDesiredTorque() ? jointDesiredSetpoints.getDesiredTorque() : 0.0;

      double desiredVelocity = jointDesiredSetpoints.hasDesiredVelocity() ? jointDesiredSetpoints.getDesiredVelocity() : 0.0;
      if (jointDesiredSetpoints.hasVelocityScaling())
         desiredVelocity *= jointDesiredSetpoints.getVelocityScaling();

      double jointDampingFeedback = jointDamping * (desiredVelocity - controllerJoint.getQd());
      this.jointDampingFeedback.set(jointDampingFeedback);
      desiredEffort += jointDampingFeedback;

      double desiredPosition = jointDesiredSetpoints.hasDesiredPosition() ? jointDesiredSetpoints.getDesiredPosition() : controllerJoint.getQ();
      double jointErrorFeedback = jointStiffness * (desiredPosition - controllerJoint.getQ());
      this.jointErrorFeedback.set(jointErrorFeedback);
      desiredEffort += jointErrorFeedback;

      this.jointEffort.set(desiredEffort);

      jointDesiredSetpoints.setDesiredTorque(desiredEffort);
   }
}
