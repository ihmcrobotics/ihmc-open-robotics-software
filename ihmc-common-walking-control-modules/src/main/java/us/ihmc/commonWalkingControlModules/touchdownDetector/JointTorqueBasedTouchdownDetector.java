package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJointBasics joint;
   private final YoDouble jointTorque;
   private final YoDouble torqueThreshold;
   private final YoDouble torqueForSureThreshold;
   private final YoBoolean touchdownDetected;
   private final YoBoolean touchdownForSureDetected;

   private final boolean dontDetectTouchdownIfAtJointLimit;
   private double signum;
   private double forSureSignum;

   public JointTorqueBasedTouchdownDetector(OneDoFJointBasics joint, YoVariableRegistry registry)
   {
      this(joint, false, registry);
   }

   /**
    * @param joint joint used to detect touchdown
    * @param dontDetectTouchdownIfAtJointLimit if true, this detector will not detect a touchdown if the joint is past a joint limit. this is to avoid
    *                                          false-positive touchdown signals given by simulated torques at joint limits
    * @param registry
    */
   public JointTorqueBasedTouchdownDetector(OneDoFJointBasics joint, boolean dontDetectTouchdownIfAtJointLimit, YoVariableRegistry registry)
   {
      this.joint = joint;
      this.dontDetectTouchdownIfAtJointLimit = dontDetectTouchdownIfAtJointLimit;

      jointTorque = new YoDouble(joint.getName() + "_torqueUsedForTouchdownDetection", registry);
      torqueThreshold = new YoDouble(joint.getName() + "_touchdownTorqueThreshold", registry);
      torqueForSureThreshold = new YoDouble(joint.getName() + "_touchdownTorqueForSureThreshold", registry);
      touchdownDetected = new YoBoolean(joint.getName() + "_torqueBasedTouchdownDetected", registry);
      touchdownForSureDetected = new YoBoolean(joint.getName() + "_torqueBasedTouchdownForSureDetected", registry);
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

   public void setTorqueForSureThreshold(double torqueThreshold)
   {
      this.torqueForSureThreshold.set(torqueThreshold);
      forSureSignum = Math.signum(torqueThreshold);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return touchdownDetected.getBooleanValue();
   }

   @Override
   public boolean hasForSureTouchedDown()
   {
      return touchdownForSureDetected.getBooleanValue();
   }

   private boolean isAtJointLimit()
   {
      double q = joint.getQ();
      double jointLimitLower = joint.getJointLimitLower();
      double jointLimitUpper = joint.getJointLimitUpper();
      return !MathTools.intervalContains(q, jointLimitLower, jointLimitUpper, false, false);
   }

   @Override
   public void update()
   {
      double threshold = torqueThreshold.getDoubleValue() * signum;
      double forSureThreshold = torqueForSureThreshold.getDoubleValue() * forSureSignum;
      double torque = joint.getTau() * signum;
      double forSureTorque = joint.getTau() * forSureSignum;

      jointTorque.set(joint.getTau());

      if (dontDetectTouchdownIfAtJointLimit && isAtJointLimit())
      {
         touchdownDetected.set(false);
         touchdownForSureDetected.set(false);
      }
      else
      {
         touchdownDetected.set(torque > threshold);
         touchdownForSureDetected.set(forSureTorque > forSureThreshold);
      }
   }

   @Override
   public void reset()
   {
      jointTorque.set(0.0);
      touchdownDetected.set(false);
      touchdownForSureDetected.set(false);
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }
}
