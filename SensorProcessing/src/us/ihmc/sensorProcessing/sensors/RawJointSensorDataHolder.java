package us.ihmc.sensorProcessing.sensors;

public class RawJointSensorDataHolder
{
   private final String name;
   
   private boolean useOutputEncoderQ = false;
   private boolean useOutputEncoderQd = false;
   private boolean isEnabled = false;
   
   private double q_raw;
   private double q_out_raw;
   private double qd_out_raw;
   private double qd_raw;
   private double f_raw;
   private double psi_neg_raw;
   private double psi_pos_raw;
   private double motorCurrent;
   private double commandedMotorCurrent;
   private double temperature;
   
   private double[] motorAngles = new double[2]; // Increase if we get more motors/joint

   public RawJointSensorDataHolder(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }
   
   public void set(RawJointSensorDataHolder source)
   {
      if(!source.getName().equals(getName()))
      {
         throw new RuntimeException("Source name does not equal name");
      }

      setIsEnabled(source.getIsEnabled());
      setQ_raw(source.getQ_raw());
      setQ_out_raw(source.getQ_out_raw());
      setQd_out_raw(source.getQd_out_raw());
      setQd_raw(source.getQd_raw());
      setF_raw(source.getF_raw());
      setPsi_neg_raw(source.getPsi_neg_raw());
      setPsi_pos_raw(source.getPsi_pos_raw());
      setUsesOutputEncoderQ(source.isUseOutputEncoderQ());
      setUsesOutputEncoderQd(source.isUseOutputEncoderQd());
      setMotorCurrent(source.getMotorCurrent());
      setCommandedMotorCurrent(source.getCommandedMotorCurrent());
      setTemperature(source.getTemperature());
      
      for (int i = 0; i < motorAngles.length; i++)
      {
         setMotorAngle(i, source.getMotorAngle(i));
      }
   }

   public boolean isUseOutputEncoderQ()
   {
      return useOutputEncoderQ;
   }
   
   
   public boolean isUseOutputEncoderQd()
   {
      return useOutputEncoderQd;
   }

   public boolean getIsEnabled()
   {
      return isEnabled;
   }

   public void setUsesOutputEncoderQ(boolean useOutputEncoder)
   {
      this.useOutputEncoderQ = useOutputEncoder;
   }

   
   public void setUsesOutputEncoderQd(boolean useOutputEncoder)
   {
      this.useOutputEncoderQd = useOutputEncoder;
   }

   public void setIsEnabled(boolean isEnabled)
   {
      this.isEnabled = isEnabled;
   }

   public double getQ_raw()
   {
      return q_raw;
   }
   
   
   public double getQd_raw()
   {
      return qd_raw;
   }

   
   public double getF_raw()
   {
      return f_raw;
   }
   
   
   public double getPsi_pos_raw()
   {
      return psi_pos_raw;
   }
   
   
   public double getPsi_neg_raw()
   {
      return psi_neg_raw;
   }
   
   
   public void setPsi_pos_raw(double psi_pos_raw)
   {
      this.psi_pos_raw = psi_pos_raw;
   }
   
   
   public void setPsi_neg_raw(double psi_neg_raw)
   {
      this.psi_neg_raw = psi_neg_raw;
   }
   
   
   public void setQ_raw(double q_raw)
   {
      this.q_raw = q_raw;
   }
   
   
   public void setQd_raw(double qd_raw)
   {
      this.qd_raw = qd_raw;
   }
   
   
   public void setF_raw(double f_raw)
   {
      this.f_raw = f_raw;
   }
   
   
   public double getQ_out_raw()
   {
      return q_out_raw;
   }
   
   
   public void setQ_out_raw(double q_out_raw)
   {
      this.q_out_raw = q_out_raw;
   }
   
   
   public double getQd_out_raw()
   {
      return qd_out_raw;
   }

   public double getTemperature()
   {
      return temperature;
   }

   public void setTemperature(double temperature)
   {
      this.temperature = temperature;
   }

   public void setQd_out_raw(double qd_out_raw)
   {
      this.qd_out_raw = qd_out_raw;
   }


   public double getMotorCurrent()
   {
      return motorCurrent;
   }


   public void setMotorCurrent(double motorCurrent)
   {
      this.motorCurrent = motorCurrent;
   }


   public double getCommandedMotorCurrent()
   {
      return commandedMotorCurrent;
   }


   public void setCommandedMotorCurrent(double commandedMotorCurrent)
   {
      this.commandedMotorCurrent = commandedMotorCurrent;
   }
   
   public void setMotorAngle(int motor, double angle)
   {
      motorAngles[motor] = angle;
   }
   
   public double getMotorAngle(int motor)
   {
      return motorAngles[motor];
   }
}
