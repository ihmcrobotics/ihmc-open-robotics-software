package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointTorqueBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final YoDouble jointTorque;
   private final YoDouble torqueThreshold;
   private final YoBoolean touchdownDetected;

   private double signum;

   public JointTorqueBasedTouchdownDetector(OneDoFJoint joint, YoVariableRegistry registry)
   {
      this.joint = joint;
      jointTorque = new YoDouble(joint.getName() + "_torqueUsedForTouchdownDetection", registry);
      torqueThreshold = new YoDouble(joint.getName() + "_touchdownTorqueThreshold", registry);
      touchdownDetected = new YoBoolean(joint.getName() + "_torqueBasedTouchdownDetected", registry);
   }

   /**
    *
    * @param torqueThreshold
    *
    * If torqueThreshold < 0, hasTouchedDown will be true when jointTorque > torqueThreshold. If toqueThreshold > 0,
    * hasTouchedDown will be true when jointTorque < torqueThreshold.
    *
    */
   public void setTorqueThreshold(double torqueThreshold)
   {
      this.torqueThreshold.set(torqueThreshold);
      signum = Math.signum(torqueThreshold);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return touchdownDetected.getBooleanValue();
   }

   @Override
   public void update()
   {
      double threshold = torqueThreshold.getDoubleValue() * signum;
      double torque = joint.getTauMeasured() * signum;

      jointTorque.set(joint.getTauMeasured());

      touchdownDetected.set(torque > threshold);
   }
}
