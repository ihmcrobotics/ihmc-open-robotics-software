package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointTorqueBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final DoubleYoVariable torqueThreshold;
   private final GlitchFilteredBooleanYoVariable touchdownDetectedFiltered;

   public JointTorqueBasedTouchdownDetector(OneDoFJoint joint, YoVariableRegistry registry)
   {
      this.joint = joint;

      torqueThreshold = new DoubleYoVariable(joint.getName() + "_touchdownTorqueThreshold", registry);
      touchdownDetectedFiltered = new GlitchFilteredBooleanYoVariable(joint.getName() + "_torqueBasedTouchdownDetected", registry, 20);
   }

   public void setTorqueThreshold(double torqueThreshold)
   {
      this.torqueThreshold.set(torqueThreshold);
   }

   @Override
   public boolean hasTouchedDown()
   {
      touchdownDetectedFiltered.update(Math.abs(joint.getTau()) > torqueThreshold.getDoubleValue());

      return touchdownDetectedFiltered.getBooleanValue();
   }
}
