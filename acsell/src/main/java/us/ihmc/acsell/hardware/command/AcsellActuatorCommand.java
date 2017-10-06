package us.ihmc.acsell.hardware.command;

import java.nio.ByteBuffer;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.steppr.hardware.StepprActuator;

public abstract class AcsellActuatorCommand
{
   private final YoVariableRegistry registry;
   private final YoBoolean enabled;
   private final YoDouble tauDesired;
   private final YoDouble tauInertia;
   //private final YoDouble currentDesired;
   private final YoDouble damping;
   private final YoDouble qddDesired;
   private final double currentLimit;
   
   private final YoDouble rawCurrentDesired;
   private final AlphaFilteredYoVariable filteredCurrentDesired;
   
   private final AcsellActuator actuator;
   
   public AcsellActuatorCommand(String name, AcsellActuator actuator, YoVariableRegistry parentRegistry)
   {
      this.actuator = actuator;
      this.currentLimit = actuator.getCurrentLimit();
      this.registry = new YoVariableRegistry(name);

      this.enabled = new YoBoolean(name + "Enabled", registry);
      this.tauDesired = new YoDouble(name + "TauDesired", registry);
      this.tauInertia = new YoDouble(name + "TauInertia", registry);
      this.qddDesired = new YoDouble(name + "qdd_d", registry);
      this.damping = new YoDouble(name + "Damping", registry);
      //this.currentDesired = new YoDouble(name+"CurrentDesired", registry);
      this.rawCurrentDesired = new YoDouble(name+"CurrentDesired", registry);
      
      if(actuator==StepprActuator.LEFT_HIP_Z || actuator==StepprActuator.RIGHT_HIP_Z)
         this.filteredCurrentDesired = new AlphaFilteredYoVariable(name+"CurrentDesired_filt", registry, 0.90, rawCurrentDesired);
      else
         this.filteredCurrentDesired = new AlphaFilteredYoVariable(name+"CurrentDesired_filt", registry, 0.0, rawCurrentDesired);
      
      rawCurrentDesired.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
        	 if(v.getValueAsDouble()>currentLimit) v.setValueFromDouble(currentLimit);
        	 if(v.getValueAsDouble()<-currentLimit) v.setValueFromDouble(-currentLimit);        	 
         }
      });
      
      parentRegistry.addChild(registry);
   }
   
   public abstract void update();
   
   

   public void write(ByteBuffer target, int controlID)
   {
      if(enabled.getBooleanValue())
      {
         target.put((byte) 3);
         target.putFloat((float) (filteredCurrentDesired.getDoubleValue()));
         target.putFloat((float) (damping.getDoubleValue() / actuator.getKt() * actuator.getDampingSign()));
         target.putFloat(0f);
         target.putInt(controlID);
      }
      else
      {
         target.put((byte) 0);
         target.putFloat(0f);
         target.putFloat(0f);
         target.putFloat(0f);
         target.putInt(controlID);
      }
   }

   public void enable()
   {
      enabled.set(true);
   }

   public void disable()
   {
      enabled.set(false);
   }
   
   public double getTauDesired()
   {
	   return tauDesired.getDoubleValue();
   }
   
   protected void setTauDesired(double tau)
   {
      tauDesired.set(tau);
      this.rawCurrentDesired.set(tau/actuator.getKt());
      this.filteredCurrentDesired.update();
   }
   
   protected void setDamping(double damping)
   {
      this.damping.set(damping);
   }
   
   protected void setQdd_d(double qdd_d)
   {
      this.qddDesired.set(qdd_d);
      this.tauInertia.set(qdd_d*actuator.getMotorInertia());
   }
}
