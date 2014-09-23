package us.ihmc.steppr.hardware.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprAnkleJointState
{
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

   // For testing purposes
   private final DenseMatrix64F wrongMatrix = new DenseMatrix64F(2, 2);

   private final DenseMatrix64F actuatorState = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F jointState = new DenseMatrix64F(2, 1);

   public StepprAnkleJointState(RobotSide robotSide, StepprActuatorState rightActuator, StepprActuatorState leftActuator, YoVariableRegistry parentRegistry)
   {
      this.leftActuator = leftActuator;
      this.rightActuator = rightActuator;

      this.registry = new YoVariableRegistry(robotSide.getCamelCaseNameForStartOfExpression() + "Ankle");

      this.q_y = new DoubleYoVariable("q_y", registry);
      this.qd_y = new DoubleYoVariable("qd_y", registry);
      this.tau_y = new DoubleYoVariable("tau_y", registry);

      this.q_x = new DoubleYoVariable("q_x", registry);
      this.qd_x = new DoubleYoVariable("qd_x", registry);
      this.tau_x = new DoubleYoVariable("tau_x", registry);

      wrongMatrix.set(0, 0, 0.5);
      wrongMatrix.set(0, 1, -0.5);
      wrongMatrix.set(1, 0, 0.5);
      wrongMatrix.set(1, 1, 0.5);
      
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      actuatorState.set(0, 0, rightActuator.getMotorPosition());
      actuatorState.set(1, 0, leftActuator.getMotorPosition());

      // The next line is fecal matter
      CommonOps.mult(wrongMatrix, actuatorState, jointState);

      this.q_y.set(jointState.get(0, 0));
      this.q_x.set(jointState.get(1, 0));
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

   }
}
