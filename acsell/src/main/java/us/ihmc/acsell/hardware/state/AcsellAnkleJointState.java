package us.ihmc.acsell.hardware.state;

import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.acsell.hardware.state.slowSensors.StrainSensor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class AcsellAnkleJointState
{	
   
   private final AcsellAnkleAngleCalculator ankleCalculator;

   private final AcsellJointState ankleX = new AnkleX();
   private final AcsellJointState ankleY = new AnkleY();

   private final AcsellActuatorState rightActuator;
   private final AcsellActuatorState leftActuator;
   
   private final StrainSensor strainSensorRight;
   private final StrainSensor strainSensorLeft;

   private final YoVariableRegistry registry;

   private final YoDouble q_y;
   private final YoDouble qd_y;
   private final YoDouble q_calc_y;
   private final YoDouble qd_calc_y;
   private final YoDouble tau_y;

   private final YoDouble q_x;
   private final YoDouble qd_x;
   private final YoDouble q_calc_x;
   private final YoDouble qd_calc_x;
   private final YoDouble tau_x;

   private final YoDouble q_rightActuator_calc;
   private final YoDouble q_leftActuator_calc;
   private final YoDouble qd_rightActuator_calc;
   private final YoDouble qd_leftActuator_calc;
   private final YoDouble tau_rightActuator;
   private final YoDouble tau_leftActuator;
   

   private final YoDouble tau_strain_Left;
   private final YoDouble tau_strain_Right;

   private final double motorAngle[] = new double[2];

   public AcsellAnkleJointState(AcsellAnkleKinematicParameters parameters, RobotSide robotSide, AcsellActuatorState rightActuator, AcsellActuatorState leftActuator, StrainSensor strainSensorRight, StrainSensor strainSensorLeft, YoVariableRegistry parentRegistry)
   {
      this.leftActuator = leftActuator;
      this.rightActuator = rightActuator;
      
      this.strainSensorRight = strainSensorRight;
      this.strainSensorLeft = strainSensorLeft;
      
      //this.ankleCalculator = new AcsellAnkleInterpolator(parameters);
      this.ankleCalculator = new AcsellAnkleFullComputation(parameters, robotSide);

      String name = robotSide.getCamelCaseNameForStartOfExpression() + "Ankle";
      this.registry = new YoVariableRegistry(name);

      this.q_y = new YoDouble(name + "_q_y", registry);
      this.qd_y = new YoDouble(name + "_qd_y", registry);
      this.q_calc_y = new YoDouble(name + "_q_calc_y", registry);
      this.qd_calc_y = new YoDouble(name + "_qd_calc_y", registry);
      this.tau_y = new YoDouble(name + "_tau_yPredictedCurrent", registry);

      this.q_x = new YoDouble(name + "_q_x", registry);
      this.qd_x = new YoDouble(name + "_qd_x", registry);
      this.q_calc_x = new YoDouble(name + "_q_calc_x", registry);
      this.qd_calc_x = new YoDouble(name + "_qd_calc_x", registry);
      this.tau_x = new YoDouble(name + "_tau_xPredictedCurrent", registry);

      this.q_leftActuator_calc = new YoDouble(name + "_q_m_leftActuator_calc", registry);
      this.q_rightActuator_calc = new YoDouble(name + "_q_m_rightActuator_calc", registry);
      this.qd_leftActuator_calc = new YoDouble(name + "_qd_m_leftActuator_calc", registry);
      this.qd_rightActuator_calc = new YoDouble(name + "_qd_m_rightActuator_calc", registry);
      this.tau_leftActuator = new YoDouble(name + "_tau_leftActuator", registry);
      this.tau_rightActuator = new YoDouble(name + "_tau_rightActuator", registry);
      
      this.tau_strain_Left = new YoDouble(name + "LeftActuator_tauMeasuredStrain", registry);
      this.tau_strain_Right = new YoDouble(name + "RightActuator_tauMeasuredStrain", registry);

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
	      
	   ankleCalculator.updateAnkleState(rightActuator,leftActuator);

	   this.q_rightActuator_calc.set(ankleCalculator.getComputedQrightActuator());
	   this.q_leftActuator_calc.set(ankleCalculator.getComputedQleftActuator());

	   this.q_calc_x.set(ankleCalculator.getComputedQAnkleX());
	   this.q_calc_y.set(ankleCalculator.getComputedQAnkleY());
	   
	   this.qd_rightActuator_calc.set(ankleCalculator.getComputedQdRightActuator());
	   this.qd_leftActuator_calc.set(ankleCalculator.getComputedQdLeftActuator());
	   
	   this.qd_calc_x.set(ankleCalculator.getComputedQdAnkleX());
	   this.qd_calc_y.set(ankleCalculator.getComputedQdAnkleY());
	       
	   this.tau_x.set(ankleCalculator.getComputedTauAnkleX());
	   this.tau_y.set(ankleCalculator.getComputedTauAnkleY());
	   
	   tau_rightActuator.set(rightActuator.getMotorTorque());
	   tau_leftActuator.set(leftActuator.getMotorTorque());
	   
	   if(strainSensorRight!=null) tau_strain_Right.set(strainSensorRight.getCalibratedValue());
      if(strainSensorLeft!=null) tau_strain_Left.set(strainSensorLeft.getCalibratedValue());
   }

   
   
   public void updateOffsets()
   {
      rightActuator.updateCanonicalAngle(q_rightActuator_calc.getDoubleValue(), 2.0 * Math.PI);
      leftActuator.updateCanonicalAngle(q_leftActuator_calc.getDoubleValue(), 2.0 * Math.PI);
   }

   public AcsellJointState ankleY()
   {
      return ankleY;
   }

   public AcsellJointState ankleX()
   {
      return ankleX;
   }

   private class AnkleY implements AcsellJointState
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
         AcsellAnkleJointState.this.update();
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
         AcsellAnkleJointState.this.updateOffsets();
      }

   }

   private class AnkleX implements AcsellJointState
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
      
      public void tareStrainGauges()
      {
   	  	if(strainSensorRight!=null) strainSensorRight.tare();
      	if(strainSensorLeft!=null) strainSensorLeft.tare();
      }

   }
}

