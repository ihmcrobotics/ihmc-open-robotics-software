package us.ihmc.acsell.hardware.command;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;

public class AcsellJointCommand
{
   private final YoVariableRegistry registry;
   
   private final YoDouble tauDesired;
   private final YoDouble damping;
   
   private final int numberOfActuators;
   private final double[] motorAngles;
   
   private double q, qd, qdd_d;
   
   
   public AcsellJointCommand(String name, int numberOfActuators, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.tauDesired = new YoDouble(name + "TauDesired", registry);
      this.damping = new YoDouble(name + "Damping", registry);
      this.numberOfActuators = numberOfActuators;
      this.motorAngles = new double[numberOfActuators];
      
      parentRegistry.addChild(registry);
   }
     
   public void setTauDesired(double tau_d, double qdd_d, RawJointSensorDataHolder rawSensorData)
   {
      this.tauDesired.set(tau_d);
      
      this.q = rawSensorData.getQ_raw();
      this.qd = rawSensorData.getQd_raw();
      this.qdd_d = qdd_d;
      for(int i = 0; i < numberOfActuators; i++)
      {
         motorAngles[i] = rawSensorData.getMotorAngle(i);
      }
   }
   
   public double getQ()
   {
      return q;
   }
   
   public double getQd()
   {
      return qd;
   }
   
   public double getQdd_d()
   {
      return qdd_d;
   }
   
   public double getTauDesired()
   {
      return tauDesired.getDoubleValue();
   }
   
   public int getNumberOfActuators()
   {
      return numberOfActuators;
   }
   
   public double getMotorAngle(int actuator)
   {
      return motorAngles[actuator];
   }
   
   public double getDamping()
   {
      return damping.getDoubleValue();
   }
   
   public void setDamping(double value)
   {
      damping.set(value);
   }
}
