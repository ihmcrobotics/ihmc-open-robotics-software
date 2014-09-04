package us.ihmc.valkyrie.kinematics.transmissions;

import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;
import us.ihmc.valkyrie.roboNet.TurboDriver;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class InefficientPushRodTransmission implements PushRodTransmissionInterface
{
   private static final double INFINITY_THRESHOLD = 1e10;
   private static final boolean DEBUG = false;

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
   private final double[][] jacobian = new double[2][2];
   private final double[][] jacobianInverse = new double[2][2];

   private final InefficientPushrodTransmissionJacobian inefficientPushrodTransmissionJacobian;
   
   private final double reflect;
   
   public InefficientPushRodTransmission(PushRodTransmissionJoint pushRodTransmissionJoint, double reflect, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (Math.abs(Math.abs(reflect) - 1.0) > 1e-7) throw new RuntimeException("reflect must be 1.0 or -1.0");
      this.reflect = reflect;
      
       inefficientPushrodTransmissionJacobian = new InefficientPushrodTransmissionJacobian(pushRodTransmissionJoint, parentRegistry, dynamicGraphicObjectsListRegistry);
   }
  
   public void setUseFuteks(boolean useFuteks)
   {
      inefficientPushrodTransmissionJacobian.setUseFuteks(useFuteks);      
   }

   private boolean invertMatrix(double[][] matrix, double[][] inverseTransposeToPack)
   {
      double determinant = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

      if (Math.abs(determinant) < 1e-5) 
      {
         printIfDebug("InefficientPushRodTransmission: determinant = " + determinant);
      }
      
      if (determinant != 0.0)
      {
         inverseTransposeToPack[0][0] = (1.0 / determinant) * matrix[1][1];
         inverseTransposeToPack[0][1] = (1.0 / determinant) * (-matrix[0][1]);
         inverseTransposeToPack[1][0] = (1.0 / determinant) * (-matrix[1][0]);
         inverseTransposeToPack[1][1] = (1.0 / determinant) * matrix[0][0];

         return true;
      }
      else
         return false;
   }
   
   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
      
   }

   // For ankles:
   // jointData[0] = ankleExtensor (Top Joint -- Pitch)
   // jointData[1] = Ankle  (Bottom Joint -- Roll)
   // actuatorData[0] = j5 (rightSide)
   // actuatorData[1] = j6 (leftSide)
   // LeftLeg has reflect = +1
   // RightLeg has reflect = -1
   private int count = 0;
   
   @Override
   public void actuatorToJointEffort(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

      TurboDriver rightTurboDriver = actuatorData[0];
      TurboDriver leftTurboDriver = actuatorData[1];
      
      ValkyrieJointInterface topJointInterface = jointData[0];
      ValkyrieJointInterface bottomJointInterface = jointData[1];
      
      double rightActuatorForce = rightTurboDriver.getEffort(); 
      double leftActuatorForce = leftTurboDriver.getEffort(); 

      double topJointAngle = topJointInterface.getPosition();
      double bottomJointAngle = reflect * bottomJointInterface.getPosition();
      
      inefficientPushrodTransmissionJacobian.computeJacobian(jacobian, topJointAngle, bottomJointAngle);

//      System.out.println("m11: " + jacobian[0][0] + ", m12: " + jacobian[0][1] + ", m21: " + jacobian[1][0] + ", m22: " + jacobian[1][1]);
      double topJointTorque = jacobian[0][0] * leftActuatorForce + jacobian[0][1] * rightActuatorForce;
      double bottomJointTorque = jacobian[1][0] *  leftActuatorForce + jacobian[1][1] * rightActuatorForce;

      topJointInterface.setEffort(topJointTorque);
      bottomJointInterface.setEffort(reflect * bottomJointTorque);
      
      if (count == 0)
      {
         System.out.println("\n\ntopJointInterface.getName() = " + topJointInterface.getName());
         System.out.println("bottomJointInterface.getName() = " + bottomJointInterface.getName());
         
         System.out.println("leftTurboDriver.getNodePath() = " + leftTurboDriver.getNodePath());
         System.out.println("rightTurboDriver.getNodePath() = " + rightTurboDriver.getNodePath());

         System.out.println("reflect = " + reflect);
         
         count++;
      }
      
   }

   @Override
   public void jointToActuatorEffort(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

      TurboDriver rightTurboDriver = actuatorData[0];
      TurboDriver leftTurboDriver = actuatorData[1];
      
      ValkyrieJointInterface topJointInterface = jointData[0];
      ValkyrieJointInterface bottomJointInterface = jointData[1];
      
      double topJointAngle = topJointInterface.getPosition();
      double bottomJointAngle = reflect * bottomJointInterface.getPosition();
      double topJointTorque = topJointInterface.getDesiredEffort();
      double bottomJointTorque = reflect * bottomJointInterface.getDesiredEffort();

      if (Math.abs(topJointAngle) > INFINITY_THRESHOLD || Math.abs(bottomJointAngle) > INFINITY_THRESHOLD)
      {
         throw new RuntimeException("jointToActuatorEffort: pitchAngle or rollAngle is infinity!!\n");
      }

      inefficientPushrodTransmissionJacobian.computeJacobian(jacobian, topJointAngle, bottomJointAngle);
      invertMatrix(jacobian, jacobianInverse);
      double leftJointForce = jacobianInverse[0][0] * topJointTorque + jacobianInverse[0][1] * bottomJointTorque;
      double rightJointForce = jacobianInverse[1][0] * topJointTorque + jacobianInverse[1][1] * bottomJointTorque;

      checkInfinity(leftJointForce);
      checkInfinity(rightJointForce);

      rightTurboDriver.setEffortCommand(rightJointForce);
      leftTurboDriver.setEffortCommand(leftJointForce);
   }

   @Override
   public void actuatorToJointVelocity(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void actuatorToJointPosition(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }
   
   
   @Override
   public void jointToActuatorVelocity(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void jointToActuatorPosition(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void updateMotorCurrents(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data)
   {
      assertTrue(numActuators() == act_data.length && numJoints() == jnt_data.length);

      jnt_data[0].setMotorCurrent(act_data[0].getCurrentIq());
      jnt_data[0].setCommandedMotorCurrent(act_data[0].getCurrentIqCmd());

      jnt_data[1].setMotorCurrent(act_data[1].getCurrentIq());
      jnt_data[1].setCommandedMotorCurrent(act_data[1].getCurrentIqCmd());
   }

}
