package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointVelocityFiniteDifferenceBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final DoubleYoVariable velocityFiniteDifference;
   private final DoubleYoVariable footInSwingThreshold, touchdownThreshold;
   private final GlitchFilteredBooleanYoVariable footInSwingFiltered;
   private final BooleanYoVariable controllerSetFootSwitch, touchdownDetected;

   private double previousVelocity;
   private boolean initialized = false;

   public JointVelocityFiniteDifferenceBasedTouchdownDetector(OneDoFJoint joint, BooleanYoVariable controllerSetFootSwitch, YoVariableRegistry registry)
   {
      this.joint = joint;
      this.controllerSetFootSwitch = controllerSetFootSwitch;

      velocityFiniteDifference = new DoubleYoVariable(joint.getName() + "_velocityFiniteDifference", registry);
      footInSwingThreshold = new DoubleYoVariable(joint.getName() + "_footInSwingThreshold", registry);
      touchdownThreshold = new DoubleYoVariable(joint.getName() + "__velocityFiniteDifferenceTouchdownThreshold", registry);
      footInSwingFiltered = new GlitchFilteredBooleanYoVariable(joint.getName() + "_footInSwingFiltered", registry, 50);
      touchdownDetected = new BooleanYoVariable(joint.getName() + "_velocityFiniteDifferenceTouchdownDetected", registry);
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
         velocityFiniteDifference.set(Math.abs(joint.getQd() - previousVelocity));
         previousVelocity = joint.getQd();

         if(!controllerSetFootSwitch.getBooleanValue())
         {
            if(footInSwingFiltered.getBooleanValue())
            {
               if(!touchdownDetected.getBooleanValue())
                  touchdownDetected.set(velocityFiniteDifference.getDoubleValue() > touchdownThreshold.getDoubleValue());
            }
            else
            {
               footInSwingFiltered.update(velocityFiniteDifference.getDoubleValue() < footInSwingThreshold.getDoubleValue());
            }
         }
         else
         {
            footInSwingFiltered.set(false);
            touchdownDetected.set(true);
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
