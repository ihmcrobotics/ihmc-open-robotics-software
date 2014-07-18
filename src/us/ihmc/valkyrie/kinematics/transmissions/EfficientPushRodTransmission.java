package us.ihmc.valkyrie.kinematics.transmissions;

import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;
import us.ihmc.valkyrie.kinematics.util.ClosedFormJacobian;
import us.ihmc.valkyrie.roboNet.TurboDriver;


public class EfficientPushRodTransmission implements PushRodTransmissionInterface
{
   private static final double INFINITY_THRESHOLD = 1e10;

   private int numActuators()
   {
      return 2;
   }

   private int numJoints()
   {
      return 2;
   }

   private void assertTrue(boolean test)
   {
      if (!test)
      {
         throw new RuntimeException();
      }
   }
   
   
   private void checkInfinity(double value)
   {
      if (Math.abs(value) > INFINITY_THRESHOLD)
      {
         throw new RuntimeException("checkInfinity: Infinity value detected in supplied data structure!");
      }
   }
   
 //Temporary matrix
   private double[][] jacobian = new double[2][2];
   private final double[][] jacobianInverse = new double[2][2];

   private final ClosedFormJacobian efficientPushrodTransmissionJacobian;
   
   private final double reflect;
   
   public EfficientPushRodTransmission(PushRodTransmissionJoint pushRodTransmissionJoint, double reflect)
   {

      if (Math.abs(Math.abs(reflect) - 1.0) > 1e-7) throw new RuntimeException("reflect must be 1.0 or -1.0");
      this.reflect = reflect;
      
      efficientPushrodTransmissionJacobian = new ClosedFormJacobian(pushRodTransmissionJoint);
   }
   
   private boolean invertMatrix(double[][] matrix, double[][] inverseTransposeToPack)
   {
      double det = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

      if (det != 0.0)
      {
         inverseTransposeToPack[0][0] = (1.0 / det) * matrix[1][1];
         inverseTransposeToPack[0][1] = (1.0 / det) * (-matrix[0][1]);
         inverseTransposeToPack[1][0] = (1.0 / det) * (-matrix[1][0]);
         inverseTransposeToPack[1][1] = (1.0 / det) * matrix[0][0];
         return true;
      }
      else
         return false;
   }

   @Override
   public void actuatorToJointEffort(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      
    assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

    double actuatorForce0 = actuatorData[0].getEffort(); //TODO: Verify the ordering here...
    double actuatorForce1 = actuatorData[1].getEffort();

    double pitchAngle = jointData[0].getPosition();
    double rollAngle = reflect * jointData[1].getPosition();
    
    jacobian = efficientPushrodTransmissionJacobian.getUpdatedTransform(rollAngle, pitchAngle);

    double rollTorque = jacobian[0][0] * actuatorForce0 + jacobian[0][1] * actuatorForce1;
    double pitchTorque  = jacobian[1][0] * actuatorForce0 + jacobian[1][1] * actuatorForce1;

    jointData[0].setEffort(pitchTorque);
    jointData[1].setEffort(reflect * rollTorque);
    
   }

  
   

   @Override
   public void actuatorToJointVelocity(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void actuatorToJointPosition(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void jointToActuatorEffort(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

      double pitchAngle = jointData[0].getPosition();
      double rollAngle = reflect * jointData[1].getPosition();
      double pitchTorque = jointData[0].getDesiredEffort();
      double rollTorque = reflect * jointData[1].getDesiredEffort();

      if (Math.abs(pitchAngle) > INFINITY_THRESHOLD || Math.abs(rollAngle) > INFINITY_THRESHOLD)
      {
         throw new RuntimeException("jointToActuatorEffort: pitchAngle or rollAngle is infinity!!\n");
      }

      jacobian = efficientPushrodTransmissionJacobian.getUpdatedTransform(rollAngle, pitchAngle);
      invertMatrix(jacobian, jacobianInverse);

      double actuatorForce0 = jacobianInverse[0][0] * rollTorque + jacobianInverse[0][1] * pitchTorque;
      double actuatorForce1 = jacobianInverse[1][0] * rollTorque + jacobianInverse[1][1] * pitchTorque;

      checkInfinity(actuatorForce0);
      checkInfinity(actuatorForce1);

      actuatorData[0].setEffortCommand(actuatorForce0); //TODO: Verify the ordering here...
      actuatorData[1].setEffortCommand(actuatorForce1);
   }

   @Override
   public void jointToActuatorVelocity(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void jointToActuatorPosition(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      // TODO Auto-generated method stub
      
   }

}
