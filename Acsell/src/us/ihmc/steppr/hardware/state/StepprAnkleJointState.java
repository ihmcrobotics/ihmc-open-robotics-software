package us.ihmc.steppr.hardware.state;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
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
   private final DoubleYoVariable tau_y;

   private final DoubleYoVariable q_x;
   private final DoubleYoVariable qd_x;
   private final DoubleYoVariable tau_x;

   private double motorAngle[] = new double[2];
   
   public StepprAnkleJointState(RobotSide robotSide, StepprActuatorState rightActuator, StepprActuatorState leftActuator, YoVariableRegistry parentRegistry)
   {
      this.leftActuator = leftActuator;
      this.rightActuator = rightActuator;

      String name = robotSide.getCamelCaseNameForStartOfExpression() + "Ankle";
      this.registry = new YoVariableRegistry(name);

      
      this.q_y = new DoubleYoVariable(name + "_q_y", registry);
      this.qd_y = new DoubleYoVariable(name + "_qd_y", registry);
      this.tau_y = new DoubleYoVariable(name + "_tau_y", registry);

      this.q_x = new DoubleYoVariable(name + "_q_x", registry);
      this.qd_x = new DoubleYoVariable(name + "_qd_x", registry);
      this.tau_x = new DoubleYoVariable(name + "_tau_x", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      motorAngle[0] = rightActuator.getMotorPosition();
      motorAngle[1] = leftActuator.getMotorPosition();
      
      interpolator.updateAnkleState(motorAngle[0], motorAngle[1], rightActuator.getMotorVelocity(),
            leftActuator.getMotorVelocity());

      this.q_x.set(interpolator.getQAnkleX());
      this.q_y.set(interpolator.getQAnkleY());

      this.qd_x.set(interpolator.getQdAnkleX());
      this.qd_y.set(interpolator.getQdAnkleY());
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
