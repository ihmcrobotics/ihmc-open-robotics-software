package us.ihmc.robotics.physics;

import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoRobotJointLimitImpulseBasedCalculator extends RobotJointLimitImpulseBasedCalculator
{
   private final YoInteger numberOfJointsAtLimit;
   private final Map<OneDoFJointReadOnly, YoJointLimitImpulseData> yoJointDataMap;

   public YoRobotJointLimitImpulseBasedCalculator(RigidBodyBasics rootBody, ForwardDynamicsCalculator forwardDynamicsCalculator, YoRegistry registry)
   {
      super(rootBody, forwardDynamicsCalculator);

      numberOfJointsAtLimit = new YoInteger("numberOfJointsAtLimit", registry);

      yoJointDataMap = SubtreeStreams.fromChildren(OneDoFJointReadOnly.class, rootBody).map(joint -> new YoJointLimitImpulseData(joint, registry))
                                     .collect(Collectors.toMap(YoJointLimitImpulseData::getJoint, Function.identity()));
   }

   @Override
   public void initialize(double dt)
   {
      super.initialize(dt);

      yoJointDataMap.forEach((joint, data) ->
      {
         data.impulse.set(0.0);
         data.jointVelocityNoImpulse.set(joint.getQd());
         data.jointVelocityDueToOtherImpulse.setToNaN();
      });
   }

   @Override
   public void finalizeImpulse()
   {
      super.finalizeImpulse();

      for (int i = 0; i < getJointTargets().size(); i++)
      {
         OneDoFJointReadOnly joint = getJointTargets().get(i);
         YoJointLimitImpulseData yoJointData = yoJointDataMap.get(joint);
         yoJointData.impulse.set(getImpulse().get(i));
         yoJointData.jointVelocityDueToOtherImpulse.set(getJointVelocityDueToOtherImpulse().get(i));
      }

      // Force update of the response calculator.
      getJointVelocityChange(0);

      yoJointDataMap.forEach((joint, data) ->
      {
         data.jointVelocityChange.set(getResponseCalculator().getJointTwistChange(joint));
      });

      numberOfJointsAtLimit.set(getJointTargets().size());
   }

   private static class YoJointLimitImpulseData
   {
      private final OneDoFJointReadOnly joint;
      private final YoDouble impulse;
      private final YoDouble jointVelocityNoImpulse;
      private final YoDouble jointVelocityDueToOtherImpulse;
      private final YoDouble jointVelocityChange;

      public YoJointLimitImpulseData(OneDoFJointReadOnly joint, YoRegistry registry)
      {
         this.joint = joint;
         impulse = new YoDouble(joint.getName() + "Impulse", registry);
         jointVelocityNoImpulse = new YoDouble(joint.getName() + "VelocityNoImpulse", registry);
         jointVelocityDueToOtherImpulse = new YoDouble(joint.getName() + "VelocityDueToOtherImpulse", registry);
         jointVelocityChange = new YoDouble(joint.getName() + "VelocityChange", registry);
      }

      public OneDoFJointReadOnly getJoint()
      {
         return joint;
      }
   }
}
