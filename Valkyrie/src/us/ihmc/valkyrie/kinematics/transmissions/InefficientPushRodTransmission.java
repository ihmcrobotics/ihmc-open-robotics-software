package us.ihmc.valkyrie.kinematics.transmissions;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.valkyrie.kinematics.LinearActuator;
import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;


public class InefficientPushRodTransmission implements PushRodTransmissionInterface
{
   private static final boolean PRINT_OUT_TRANSMISSION_INFORMATION_ON_FIRST_CALL = false;
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

   private final PushrodTransmissionJacobian pushrodTransmissionJacobian;
   
   private final double reflectBottom;
   private final double reflectTop;
   private final boolean topJointFirst;
   
   private DoubleYoVariable topJointAngleOffset;
   private final PushRodTransmissionJoint pushRodTransmissionJoint;
   
   /*
    *  for the waist we have:
    *  reflectTop = -1.0
    *  reflectBottom = 1.0
    *  topJointFirst = false
    */
   
   public InefficientPushRodTransmission(PushRodTransmissionJoint pushRodTransmissionJoint, 
			double reflectTop, double reflectBottom, boolean topJointFirst,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.pushRodTransmissionJoint = pushRodTransmissionJoint;
      
      if (Math.abs(Math.abs(reflectBottom) - 1.0) > 1e-7) throw new RuntimeException("reflect must be 1.0 or -1.0");
      this.reflectBottom = reflectBottom;
      this.reflectTop = reflectTop;
      this.topJointFirst = topJointFirst;
      
       pushrodTransmissionJacobian = new InefficientPushrodTransmissionJacobian(pushRodTransmissionJoint, parentRegistry, yoGraphicsListRegistry);
   }
   
   public void allowTopJointAngleOffset(String namePrefix, double offset, YoVariableRegistry registry)
   {
      topJointAngleOffset = new DoubleYoVariable(namePrefix + "TopJointAngleOffset", registry);
      topJointAngleOffset.set(offset);
   }
  
   public void setUseFuteks(boolean useFuteks)
   {
      pushrodTransmissionJacobian.setUseFuteks(useFuteks);
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

   private void printOutTransmissionInformation(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      System.out.println("\nInefficientPushrodTransmission for " + pushRodTransmissionJoint);
      System.out.println("reflectTop = " + reflectTop);
      System.out.println("reflectBottom = " + reflectBottom);
      System.out.println("topJointFirst = " + topJointFirst);
      
      if (topJointAngleOffset != null) System.out.println("topJointAngleOffset = " + topJointAngleOffset.getDoubleValue());
      
      System.out.println("TurboDrivers: ");

      for (LinearActuator truboDriver : actuatorData)
      {
         System.out.println(truboDriver.getName());
      }
      
      System.out.println("ValkyrieJointInterfaces: ");

      for (ValkyrieJointInterface valkyrieJointInterface : jointData)
      {
         System.out.println(valkyrieJointInterface.getName());
      }
   }
   
   
   // For ankles:
   // jointData[0] = ankleExtensor (Top Joint -- Pitch)
   // jointData[1] = Ankle  (Bottom Joint -- Roll)
   // actuatorData[0] = j5 (rightSide)
   // actuatorData[1] = j6 (leftSide)
   // LeftLeg has reflect = +1
   // RightLeg has reflect = -1
   
   private boolean printOutAtuatorToJointEffort = PRINT_OUT_TRANSMISSION_INFORMATION_ON_FIRST_CALL;
   
   @Override
   public void actuatorToJointEffort(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      if(printOutAtuatorToJointEffort)
      {
         System.out.println("\nactuatorToJointEffort():");
         printOutTransmissionInformation(actuatorData, jointData);
         printOutAtuatorToJointEffort = false;
      }
      
      assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

      LinearActuator rightTurboDriver = actuatorData[0];
      LinearActuator leftTurboDriver = actuatorData[1];
      
      ValkyrieJointInterface topJointInterface, bottomJointInterface;

      if (topJointFirst)
      {
         topJointInterface = jointData[0];
         bottomJointInterface = jointData[1];
      }
      else
      {
         topJointInterface = jointData[1];
         bottomJointInterface = jointData[0];
      }
      
      double rightActuatorForce = rightTurboDriver.getEffort(); 
      double leftActuatorForce = leftTurboDriver.getEffort(); 

      double topJointAngle = reflectTop * topJointInterface.getPosition();
      if (topJointAngleOffset != null) topJointAngle += topJointAngleOffset.getDoubleValue();
      double bottomJointAngle = reflectBottom * bottomJointInterface.getPosition();
      
      pushrodTransmissionJacobian.computeJacobian(jacobian, topJointAngle, bottomJointAngle);

//      System.out.println("m11: " + jacobian[0][0] + ", m12: " + jacobian[0][1] + ", m21: " + jacobian[1][0] + ", m22: " + jacobian[1][1]);
      double topJointTorque = jacobian[0][0] * leftActuatorForce + jacobian[0][1] * rightActuatorForce;
      double bottomJointTorque = jacobian[1][0] *  leftActuatorForce + jacobian[1][1] * rightActuatorForce;

      topJointInterface.setEffort(reflectTop * topJointTorque);
      bottomJointInterface.setEffort(reflectBottom * bottomJointTorque);
   }

   private boolean printOutActuatorEffortHasBeenCalled = PRINT_OUT_TRANSMISSION_INFORMATION_ON_FIRST_CALL;
   
   @Override
   public void jointToActuatorEffort(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      if(printOutActuatorEffortHasBeenCalled)
      {
         System.out.println("\njointToActuatorEffort():");
         printOutTransmissionInformation(actuatorData, jointData);
         printOutActuatorEffortHasBeenCalled = false;
      }
      
      assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

      LinearActuator rightTurboDriver = actuatorData[0];
      LinearActuator leftTurboDriver = actuatorData[1];
      
      ValkyrieJointInterface topJointInterface, bottomJointInterface;

      if (topJointFirst)
      {
         topJointInterface = jointData[0];
         bottomJointInterface = jointData[1];
      }
      else
      {
         topJointInterface = jointData[1];
         bottomJointInterface = jointData[0];
      }
      
      double topJointAngle = reflectTop * topJointInterface.getPosition();
      double bottomJointAngle = reflectBottom * bottomJointInterface.getPosition();
      if (topJointAngleOffset != null) topJointAngle += topJointAngleOffset.getDoubleValue();

      double topJointTorque = reflectTop * topJointInterface.getDesiredEffort();
      double bottomJointTorque = reflectBottom * bottomJointInterface.getDesiredEffort();

      if (Math.abs(topJointAngle) > INFINITY_THRESHOLD || Math.abs(bottomJointAngle) > INFINITY_THRESHOLD)
      {
         throw new RuntimeException("jointToActuatorEffort: pitchAngle or rollAngle is infinity!!\n");
      }

      pushrodTransmissionJacobian.computeJacobian(jacobian, topJointAngle, bottomJointAngle);
      invertMatrix(jacobian, jacobianInverse);
      double leftJointForce = jacobianInverse[0][0] * topJointTorque + jacobianInverse[0][1] * bottomJointTorque;
      double rightJointForce = jacobianInverse[1][0] * topJointTorque + jacobianInverse[1][1] * bottomJointTorque;

      checkInfinity(leftJointForce);
      checkInfinity(rightJointForce);

      rightTurboDriver.setEffortCommand(rightJointForce);
      leftTurboDriver.setEffortCommand(leftJointForce);
   }
   
   public double[] jointToActuatorEffortForTorqueOffsets(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      if(printOutActuatorEffortHasBeenCalled)
      {
         System.out.println("\njointToActuatorEffort():");
         printOutTransmissionInformation(actuatorData, jointData);
         printOutActuatorEffortHasBeenCalled = false;
      }
      
      assertTrue(numActuators() == actuatorData.length && numJoints() == jointData.length);

      LinearActuator rightTurboDriver = actuatorData[0];
      LinearActuator leftTurboDriver = actuatorData[1];
      
      ValkyrieJointInterface topJointInterface, bottomJointInterface;

      if (topJointFirst)
      {
         topJointInterface = jointData[0];
         bottomJointInterface = jointData[1];
      }
      else
      {
         topJointInterface = jointData[1];
         bottomJointInterface = jointData[0];
      }
      
      double topJointAngle = reflectTop * topJointInterface.getPosition();
      double bottomJointAngle = reflectBottom * bottomJointInterface.getPosition();
      if (topJointAngleOffset != null) topJointAngle += topJointAngleOffset.getDoubleValue();

      double topJointTorque = reflectTop * topJointInterface.getEffort();
      double bottomJointTorque = reflectBottom * bottomJointInterface.getEffort();

      if (Math.abs(topJointAngle) > INFINITY_THRESHOLD || Math.abs(bottomJointAngle) > INFINITY_THRESHOLD)
      {
         throw new RuntimeException("jointToActuatorEffort: pitchAngle or rollAngle is infinity!!\n");
      }

      pushrodTransmissionJacobian.computeJacobian(jacobian, topJointAngle, bottomJointAngle);
      invertMatrix(jacobian, jacobianInverse);
      double leftJointForce = jacobianInverse[0][0] * topJointTorque + jacobianInverse[0][1] * bottomJointTorque;
      double rightJointForce = jacobianInverse[1][0] * topJointTorque + jacobianInverse[1][1] * bottomJointTorque;

      checkInfinity(leftJointForce);
      checkInfinity(rightJointForce);

      return new double[]{rightJointForce, leftJointForce};   
   }
   
   public double[] jointToActuatorEffortAtZero(double[] jointTorques)
   {
      double topJointAngle = 0.0;
      double bottomJointAngle = 0.0;

      double topJointTorque = reflectTop * jointTorques[0];
      double bottomJointTorque = reflectBottom * jointTorques[1];

      pushrodTransmissionJacobian.computeJacobian(jacobian, topJointAngle, bottomJointAngle);
      invertMatrix(jacobian, jacobianInverse);
      double leftJointForce = jacobianInverse[0][0] * topJointTorque + jacobianInverse[0][1] * bottomJointTorque;
      double rightJointForce = jacobianInverse[1][0] * topJointTorque + jacobianInverse[1][1] * bottomJointTorque;

      checkInfinity(leftJointForce);
      checkInfinity(rightJointForce);

      return new double[]{rightJointForce, leftJointForce};
   }

   @Override
   public void actuatorToJointVelocity(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void actuatorToJointPosition(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }
   
   
   @Override
   public void jointToActuatorVelocity(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void jointToActuatorPosition(LinearActuator[] actuatorData, ValkyrieJointInterface[] jointData)
   {
      // TODO Auto-generated method stub
      
   }
}
