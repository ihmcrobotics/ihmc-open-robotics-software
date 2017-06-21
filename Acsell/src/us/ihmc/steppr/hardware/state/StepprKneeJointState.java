package us.ihmc.steppr.hardware.state;

import us.ihmc.acsell.hardware.state.AcsellActuatorState;
import us.ihmc.acsell.hardware.state.AcsellJointState;
import us.ihmc.acsell.hardware.state.slowSensors.StrainSensor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.steppr.hardware.StepprJoint;

public class StepprKneeJointState implements AcsellJointState
{
   private final YoVariableRegistry registry;

   private final AcsellActuatorState actuator;
   private final AcsellActuatorState ankle;
   private final StrainSensor strainSensor;
   

   private double ratio;

   private double motorAngle;
   
   private final YoDouble q;
   private final YoDouble qd;
   private final YoDouble tau_current;
   private final YoDouble tau_strain;

   public StepprKneeJointState(StepprJoint joint, AcsellActuatorState actuator, AcsellActuatorState ankle, StrainSensor strainSensor, YoVariableRegistry parentRegistry)
   {
      String name = joint.getSdfName();
      this.registry = new YoVariableRegistry(name);
      this.ratio = joint.getRatio();
      this.actuator = actuator;
      this.ankle = ankle;
      this.strainSensor = strainSensor;
      
      
      this.q = new YoDouble(name + "_q", registry);
      this.qd = new YoDouble(name + "_qd", registry);
      this.tau_current = new YoDouble(name + "_tauPredictedCurrent", registry);
      this.tau_strain = new YoDouble(name + "_tauMeasuredStrain", registry);
      
      parentRegistry.addChild(registry);
   }


   @Override
   public double getQ()
   {
      return q.getDoubleValue();
   }

   @Override
   public double getQd()
   {
      return qd.getDoubleValue();
   }

   @Override
   public double getTau()
   {
      return tau_current.getDoubleValue();
   }

   @Override
   public void update()
   {
      
      final long toleranceWindowSize=1;
      if(ankle.getConsecutivePacketDropCount()<=toleranceWindowSize && actuator.getConsecutivePacketDropCount()<=toleranceWindowSize)
      {
         double ankleAngle = ankle.getMotorPosition();
         double ankleVelocity = ankle.getMotorVelocity();      
         q.set(AngleTools.trimAngleMinusPiToPi(actuator.getJointPosition() + ankleAngle));
         qd.set(actuator.getJointVelocity() + ankleVelocity);
      }
      
      motorAngle = actuator.getMotorPosition();      
      tau_current.set(actuator.getMotorTorque() * ratio);
      tau_strain.set(strainSensor.getCalibratedValue());
   }

   @Override
   public int getNumberOfActuators()
   {
      return 1;
   }

   @Override
   public double getMotorAngle(int actuator)
   {
      return motorAngle;
   }


   @Override
   public void updateOffsets()
   {
   }

}
