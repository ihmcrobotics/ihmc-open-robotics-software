package us.ihmc.steppr.hardware.state;

import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprAnkleJointState
{	
   
   private final StepprAnkleAngleCalculator interpolator = new StepprAnkleInterpolator();

   private final StepprJointState ankleX = new AnkleX();
   private final StepprJointState ankleY = new AnkleY();

   private final StepprActuatorState rightActuator;
   private final StepprActuatorState leftActuator;

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
   private final DoubleYoVariable tau_x;

   private final DoubleYoVariable q_rightActuator_calc;
   private final DoubleYoVariable q_leftActuator_calc;
   private final DoubleYoVariable qd_rightActuator_calc;
   private final DoubleYoVariable qd_leftActuator_calc;

   private final double motorAngle[] = new double[2];

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
      this.tau_y = new DoubleYoVariable(name + "_tau_yPredictedCurrent", registry);

      this.q_x = new DoubleYoVariable(name + "_q_x", registry);
      this.qd_x = new DoubleYoVariable(name + "_qd_x", registry);
      this.q_calc_x = new DoubleYoVariable(name + "_q_calc_x", registry);
      this.qd_calc_x = new DoubleYoVariable(name + "_qd_calc_x", registry);
      this.tau_x = new DoubleYoVariable(name + "_tau_xPredictedCurrent", registry);

      this.q_leftActuator_calc = new DoubleYoVariable(name + "_q_m_leftActuator_calc", registry);
      this.q_rightActuator_calc = new DoubleYoVariable(name + "_q_m_rightActuator_calc", registry);
      this.qd_leftActuator_calc = new DoubleYoVariable(name + "_qd_m_leftActuator_calc", registry);
      this.qd_rightActuator_calc = new DoubleYoVariable(name + "_qd_m_crightActuator_calc", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
	   
	   motorAngle[0] = rightActuator.getMotorPosition();
	   motorAngle[1] = leftActuator.getMotorPosition();
	   
	   //joint encoder associated with right actuator measures ankle x
	   //joint encoder associated with left actuator measures ankle y
	   this.q_x.set(AngleTools.trimAngleMinusPiToPi(rightActuator.getJointPosition()));
	   this.qd_x.set(rightActuator.getJointVelocity());
	   this.q_y.set(AngleTools.trimAngleMinusPiToPi(leftActuator.getJointPosition()));
	   this.qd_y.set(leftActuator.getJointVelocity());
	      
	   interpolator.updateAnkleState(rightActuator,leftActuator);

	   this.q_rightActuator_calc.set(interpolator.getComputedQrightActuator());
	   this.q_leftActuator_calc.set(interpolator.getComputedQleftActuator());

	   this.q_calc_x.set(interpolator.getComputedQAnkleX());
	   this.q_calc_y.set(interpolator.getComputedQAnkleY());
	   
	   this.qd_rightActuator_calc.set(interpolator.getComputedQdRightActuator());
	   this.qd_leftActuator_calc.set(interpolator.getComputedQdLeftActuator());
	   
	   this.qd_calc_x.set(interpolator.getComputedQdAnkleX());
	   this.qd_calc_y.set(interpolator.getComputedQdAnkleY());
	       
	   this.tau_x.set(interpolator.getComputedTauAnkleX());
	   this.tau_y.set(interpolator.getComputedTauAnkleY());
     
   }

   
   
   public void updateOffsets()
   {
      rightActuator.updateCanonicalAngle(q_rightActuator_calc.getDoubleValue(), 2.0 * Math.PI);
      leftActuator.updateCanonicalAngle(q_leftActuator_calc.getDoubleValue(), 2.0 * Math.PI);
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

      @Override
      public void updateOffsets()
      {
         StepprAnkleJointState.this.updateOffsets();
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

      @Override
      public void updateOffsets()
      {
         // Offset is already updated by ankle Y.
      }

   }
}
