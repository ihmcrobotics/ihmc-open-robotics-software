package us.ihmc.steppr.hardware.state;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprAnkleJointState
{
   private final StepprAnkleAngleCalculator interpolator = new StepprAnkleInterpolator();

   private final StepprJointState ankleY = new AnkleY();
   private final StepprJointState ankleX = new AnkleX();

   private final StepprActuatorState leftActuator;
   private final StepprActuatorState rightActuator;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable q_y;
   private final DoubleYoVariable qd_y;
   private final DoubleYoVariable q_calc_y;
   private final DoubleYoVariable qd_calc_y;
   private final DoubleYoVariable tau_y;

   private final DoubleYoVariable q_x;
   private final DoubleYoVariable qd_x;
   
   private final DoubleYoVariable q_calc_x;
   private final DoubleYoVariable qd_calc_x;
   
   private final DoubleYoVariable q_m_calc_leftActuator;
   private final DoubleYoVariable q_m_calc_rightActuator;
   
   private final DoubleYoVariable tau_x;

   private double motorAngle[] = new double[2];
   
   private final BooleanYoVariable setOffset;
   
   public StepprAnkleJointState(RobotSide robotSide, StepprActuatorState rightActuator, StepprActuatorState leftActuator, YoVariableRegistry parentRegistry)
   {
      this.leftActuator = leftActuator;
      this.rightActuator = rightActuator;

      String name = robotSide.getCamelCaseNameForStartOfExpression() + "Ankle";
      this.registry = new YoVariableRegistry(name);

      
      this.q_y = new DoubleYoVariable(name + "_q_y", registry);
      this.qd_y = new DoubleYoVariable(name + "_qd_y", registry);
      this.q_calc_y = new DoubleYoVariable(name + "_q_calc_y", registry);
      this.qd_calc_y = new DoubleYoVariable(name + "_qd_calc_y", registry);
      this.tau_y = new DoubleYoVariable(name + "_tau_y", registry);

      this.q_x = new DoubleYoVariable(name + "_q_x", registry);
      this.qd_x = new DoubleYoVariable(name + "_qd_x", registry);
      this.q_calc_x = new DoubleYoVariable(name + "_q_calc_x", registry);
      this.qd_calc_x = new DoubleYoVariable(name + "_qd_calc_x", registry);
      this.tau_x = new DoubleYoVariable(name + "_tau_x", registry);
      
      this.q_m_calc_leftActuator = new DoubleYoVariable(name + "_q_m_calc_leftActuator", registry);
      this.q_m_calc_rightActuator = new DoubleYoVariable(name + "_q_m_calc_rightActuator", registry);

      
      this.setOffset = new BooleanYoVariable(name + "SetOffset", registry);
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      motorAngle[0] = rightActuator.getMotorPosition();
      motorAngle[1] = leftActuator.getMotorPosition();
      
      interpolator.updateAnkleState(motorAngle[0], motorAngle[1], rightActuator.getMotorVelocity(),
            leftActuator.getMotorVelocity());

      this.q_calc_x.set(interpolator.getQAnkleX());
      this.q_calc_y.set(interpolator.getQAnkleY());

      this.qd_calc_x.set(interpolator.getQdAnkleX());
      this.qd_calc_y.set(interpolator.getQdAnkleY());
      
      this.q_x.set(rightActuator.getJointPosition());
      this.qd_x.set(rightActuator.getJointVelocity());
      this.q_y.set(leftActuator.getJointPosition());
      this.qd_y.set(leftActuator.getJointVelocity());
      
      //TODO: Check right/left
      this.q_m_calc_rightActuator.set(interpolator.calculateMotor1Angle(this.q_x.getDoubleValue(), this.q_y.getDoubleValue()));
      this.q_m_calc_leftActuator.set(interpolator.calculateMotor2Angle(this.q_x.getDoubleValue(), this.q_y.getDoubleValue()));
      
      if(setOffset.getBooleanValue())
      {
         rightActuator.updateCanonicalAngle(motorAngle[0], 2.0 * Math.PI / 6.0);
         leftActuator.updateCanonicalAngle(motorAngle[1], 2.0 * Math.PI / 6.0);
         setOffset.set(false);
      }
   }

   public StepprJointState ankleY()
   {
      return ankleY;
   }

   public StepprJointState ankleX()
   {
      return ankleX;
   }

   private class AnkleY implements StepprJointState
   {

      @Override
      public double getQ()
      {
         return q_y.getDoubleValue();
      }

      @Override
      public double getQd()
      {
         return qd_y.getDoubleValue();
      }

      @Override
      public double getTau()
      {
         return tau_y.getDoubleValue();
      }

      @Override
      public void update()
      {
         StepprAnkleJointState.this.update();
      }

      @Override
      public int getNumberOfActuators()
      {
         return 2;
      }

      @Override
      public double getMotorAngle(int actuator)
      {
         return motorAngle[actuator];
      }

   }

   private class AnkleX implements StepprJointState
   {

      @Override
      public double getQ()
      {
         return q_x.getDoubleValue();
      }

      @Override
      public double getQd()
      {
         return qd_x.getDoubleValue();
      }

      @Override
      public double getTau()
      {
         return tau_x.getDoubleValue();
      }

      @Override
      public void update()
      {
         // State is already updated by ankle Y.
      }

      @Override
      public int getNumberOfActuators()
      {
         return 2;
      }

      @Override
      public double getMotorAngle(int actuator)
      {
         return motorAngle[actuator];
      }

   }
}
