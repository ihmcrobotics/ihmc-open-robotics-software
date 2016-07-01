package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointTorqueBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final DoubleYoVariable torqueThreshold;
   private final BooleanYoVariable touchdownDetected;

   private double previousTau;

   public JointTorqueBasedTouchdownDetector(OneDoFJoint joint, YoVariableRegistry registry)
   {
      this.joint = joint;

      torqueThreshold = new DoubleYoVariable(joint.getName() + "_touchdownTorqueThreshold", registry);
      touchdownDetected = new BooleanYoVariable(joint.getName() + "_torqueBasedTouchdownDetected", registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      touchdownDetected.set(joint.getTau() - previousTau > torqueThreshold.getDoubleValue());
      previousTau = joint.getTau();

      return touchdownDetected.getBooleanValue();
   }
}
