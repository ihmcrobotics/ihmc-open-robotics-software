package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.BooleanYoVariable;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointVelocityFiniteDifferenceBasedTouchdownDetector implements TouchdownDetector
{
   private final OneDoFJoint joint;
   private final AlphaFilteredYoVariable velocityFiniteDifferenceFiltered;
   private final DoubleYoVariable footInSwingThreshold, touchdownThreshold, finiteDifferenceAlphaFilter;
   private final GlitchFilteredBooleanYoVariable footInSwingFiltered;
   private final BooleanYoVariable controllerSetFootSwitch, touchdownDetected;

   private double previousVelocity;
   private boolean initialized = false;

   public JointVelocityFiniteDifferenceBasedTouchdownDetector(OneDoFJoint joint, BooleanYoVariable controllerSetFootSwitch, YoVariableRegistry registry)
   {
      this.joint = joint;
      this.controllerSetFootSwitch = controllerSetFootSwitch;

      finiteDifferenceAlphaFilter = new DoubleYoVariable(joint.getName() + "_velocityFiniteDifferenceAlpha", registry);
      finiteDifferenceAlphaFilter.set(0.99);
      velocityFiniteDifferenceFiltered = new AlphaFilteredYoVariable(joint.getName() + "_velocityFiniteDifferenceFiltered", registry, finiteDifferenceAlphaFilter);
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
      return touchdownDetected.getBooleanValue();
   }

   @Override
   public void update()
   {
      if(initialized)
      {
         velocityFiniteDifferenceFiltered.update(Math.abs(joint.getQd() - previousVelocity) * 1000);
         previousVelocity = joint.getQd();

         if(!controllerSetFootSwitch.getBooleanValue() || !touchdownDetected.getBooleanValue())
         {
            footInSwingFiltered.update(velocityFiniteDifferenceFiltered.getDoubleValue() < footInSwingThreshold.getDoubleValue());
            if(footInSwingFiltered.getBooleanValue())
            {
               touchdownDetected.set(velocityFiniteDifferenceFiltered.getDoubleValue() > touchdownThreshold.getDoubleValue());
            }
         }
         else
         {
            footInSwingFiltered.set(false);
         }
      }
      else
      {
         previousVelocity = joint.getQd();
         initialized = true;
      }
   }
}
