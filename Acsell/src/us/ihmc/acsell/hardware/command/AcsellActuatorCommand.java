package us.ihmc.acsell.hardware.command;

import java.nio.ByteBuffer;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public abstract class AcsellActuatorCommand
{
   private final YoVariableRegistry registry;
   private final BooleanYoVariable enabled;
   private final DoubleYoVariable tauDesired;
   private final DoubleYoVariable tauInertia;
   private final DoubleYoVariable currentDesired;
   private final DoubleYoVariable damping;
   private final DoubleYoVariable qddDesired;
   
   private final AcsellActuator actuator;
   
   public AcsellActuatorCommand(String name, AcsellActuator actuator, YoVariableRegistry parentRegistry)
   {
      this.actuator = actuator;
      this.registry = new YoVariableRegistry(name);

      this.enabled = new BooleanYoVariable(name + "Enabled", registry);
      this.tauDesired = new DoubleYoVariable(name + "TauDesired", registry);
      this.tauInertia = new DoubleYoVariable(name + "TauInertia", registry);
      this.qddDesired = new DoubleYoVariable(name + "qdd_d", registry);
      this.damping = new DoubleYoVariable(name + "Damping", registry);
      this.currentDesired = new DoubleYoVariable(name+"CurrentDesired", registry);
      
      currentDesired.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
        	 final double maxCurrent = 22.0;
        	 if(v.getValueAsDouble()>maxCurrent) v.setValueFromDouble(maxCurrent);
        	 if(v.getValueAsDouble()<-maxCurrent) v.setValueFromDouble(-maxCurrent);        	 
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
         target.putFloat((float) (currentDesired.getDoubleValue()));
         target.putFloat((float) (damping.getDoubleValue() / actuator.getKt()));
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
      this.currentDesired.set(tau/actuator.getKt());
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
