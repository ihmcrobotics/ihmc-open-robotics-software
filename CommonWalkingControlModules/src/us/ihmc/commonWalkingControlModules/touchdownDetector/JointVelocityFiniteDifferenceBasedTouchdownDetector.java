package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointVelocityFiniteDifferenceBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final DoubleYoVariable velocityFiniteDifference;
   private final DoubleYoVariable liftOffThreshold, footInSwingThreshold, touchdownThreshold;
   private final BooleanYoVariable controllerSetFootSwitch, liftOffDetected, footInSwing, touchdownDetected;

   private double previousVelocity;
   private boolean initialized = false;

   public JointVelocityFiniteDifferenceBasedTouchdownDetector(OneDoFJoint joint, BooleanYoVariable controllerSetFootSwitch, YoVariableRegistry registry)
   {
      this.joint = joint;
      this.controllerSetFootSwitch = controllerSetFootSwitch;

      velocityFiniteDifference = new DoubleYoVariable(joint.getName() + "_velocityFiniteDifference", registry);
      liftOffThreshold = new DoubleYoVariable(joint.getName() + "_footLiftOffThreshold", registry);
      footInSwingThreshold = new DoubleYoVariable(joint.getName() + "_footInSwingThreshold", registry);
      touchdownThreshold = new DoubleYoVariable(joint.getName() + "__velocityFiniteDifferenceTouchdownThreshold", registry);
      liftOffDetected = new BooleanYoVariable(joint.getName() + "_footLiftOffDetected", registry);
      footInSwing = new BooleanYoVariable(joint.getName() + "_footInSwing", registry);
      touchdownDetected = new BooleanYoVariable(joint.getName() + "_velocityFiniteDifferenceTouchdownDetected", registry);
   }

   public void setLiftOffThreshold(double liftOffThreshold)
   {
      this.liftOffThreshold.set(liftOffThreshold);
   }

   public void setFootInSwingThreshold(double footInSwingThreshold)
   {
      this.footInSwingThreshold.set(footInSwingThreshold);
   }

   public void setTouchdownThreshold(double touchdownThreshold)
   {
      this.touchdownThreshold.set(touchdownThreshold);
   }

   @Override
   public boolean hasTouchedDown()
   {
      if(initialized)
      {
         velocityFiniteDifference.set(joint.getQd() - previousVelocity);
         previousVelocity = joint.getQd();

         if(!controllerSetFootSwitch.getBooleanValue())
         {
            if(liftOffDetected.getBooleanValue())
            {
               if(footInSwing.getBooleanValue())
               {
                  if(!touchdownDetected.getBooleanValue())
                     touchdownDetected.set(velocityFiniteDifference.getDoubleValue() < touchdownThreshold.getDoubleValue());
               }
               else
               {
                  footInSwing.set(Math.abs(velocityFiniteDifference.getDoubleValue()) < footInSwingThreshold.getDoubleValue());
               }
            }
            else
            {
               liftOffDetected.set(velocityFiniteDifference.getDoubleValue() < liftOffThreshold.getDoubleValue());
            }
         }
         else
         {
            liftOffDetected.set(false);
            footInSwing.set(false);
            touchdownDetected.set(false);
         }
      }
      else
      {
         previousVelocity = joint.getQd();
         initialized = true;
      }

      return touchdownDetected.getBooleanValue();
   }
}
