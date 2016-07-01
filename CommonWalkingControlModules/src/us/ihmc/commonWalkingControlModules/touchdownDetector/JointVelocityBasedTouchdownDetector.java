package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointVelocityBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final DoubleYoVariable velocityThreshold;
   private final BooleanYoVariable touchdownDetected;

   public JointVelocityBasedTouchdownDetector(OneDoFJoint joint, YoVariableRegistry registry)
   {
      this.joint = joint;

      velocityThreshold = new DoubleYoVariable(joint.getName() + "_touchdownVelocityThreshold", registry);
      touchdownDetected = new BooleanYoVariable(joint.getName() + "_velocityBasedTouchdownDetected", registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      touchdownDetected.set(joint.getQd() > velocityThreshold.getDoubleValue());

      return touchdownDetected.getBooleanValue();
   }
}
