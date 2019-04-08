package us.ihmc.valkyrie.kinematics.util;

//~--- non-JDK imports --------------------------------------------------------
import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.valkyrie.kinematics.transmissions.InefficientPushrodTransmissionJacobian;
import us.ihmc.valkyrie.kinematics.transmissions.PushRodTransmissionJoint;

public class ClosedFormJacobianTest
{
   private static final boolean DEBUG = false;
   private final double TOLERANCE = 1E-7;
   private final double TOLERANCE_GOOD_ENOUGH_FOR_GOVERNMENT_WORK = 2E-3;
   private double[] roll = new double[] { 0.0, 0.1, 0.1, -0.1, 0.0, 0.0, 0.25 };
   private double[] pitch = new double[] { 0.0, 0.2, -0.2, 0.2, 0.35, -0.35, 0.0 };
   private ClosedFormJacobian closedFormJacobianAnkle = new ClosedFormJacobian(PushRodTransmissionJoint.ANKLE);
   private ClosedFormJacobian closedFormJacobianAnkleRenishaws = new ClosedFormJacobian(PushRodTransmissionJoint.ANKLE);
   private ClosedFormJacobian closedFormJacobianWaist = new ClosedFormJacobian(PushRodTransmissionJoint.WAIST);
   private ClosedFormJacobian closedFormJacobianWaistRenishaws = new ClosedFormJacobian(PushRodTransmissionJoint.WAIST);
   private double[] m11_matlab = new double[] { -0.0341186865059837, -0.0298823091357904, -0.0318160782835614, -0.0344018257826992, -0.0303559109244497, -0.0334135464455588, -0.0267978328143697 };
   private double[] m12_matlab = new double[] { 0.0341186865059837, 0.0344018257826992, 0.0364053846119558, 0.0298823091357904, 0.0303559109244497, 0.0334135464455588, 0.0381505409007311 };
   private double[] m21_matlab = new double[] { 0.0366712094326246, 0.0369525474192938, 0.0355099629333052, 0.0347403295453367, 0.0344099137953029, 0.0340672933874358, 0.0376791531319578 };
   private double[] m22_matlab = new double[] { 0.0366712094326246, 0.0347403295453367, 0.0362023558757294, 0.0369525474192938, 0.0344099137953029, 0.0340672933874358, 0.0353981354095287 };
   private double[] m11_matlab_waist = new double[] { 0.063367872788363, 0.070375770098200, 0.058422901012790, 0.067299533258194, 0.072602680754717, 0.052124598829027, 0.065948809462726 };
   private double[] m12_matlab_waist = new double[] { 0.063367872788363, 0.067299533258194, 0.055151262022775, 0.070375770098200, 0.072602680754717, 0.052124598829027, 0.057675420837286 };
   private double[] m21_matlab_waist = new double[] { -0.045200359335076, -0.042531256340934, -0.047683204352025, -0.040960816370066, -0.039979762559698, -0.051849035209403, -0.043582905494040 };
   private double[] m22_matlab_waist = new double[] { 0.045200359335076, 0.040960816370066, 0.049961508736648, 0.042531256340934, 0.039979762559698, 0.051849035209403, 0.044875400766635 };

   @Test
   public void testJacobianMatchesMATLABAnkle()
   {
      for (int i = 0; i < 7; i++)
      {
         double[][] efficientJacobian = closedFormJacobianAnkle.getUpdatedTransform(-roll[i], -pitch[i]);
         double[][] matlabJacobian = new double[2][2];

         matlabJacobian[0][0] = -m21_matlab[i];
         matlabJacobian[0][1] = -m11_matlab[i];
         matlabJacobian[1][0] = -m22_matlab[i];
         matlabJacobian[1][1] = -m12_matlab[i];

         compareMatrices(efficientJacobian, matlabJacobian, TOLERANCE);
      }
   }

	@Disabled
   @Test
   public void testJacobianMatchesMATLABWaist()
   {
      for (int i = 0; i < 7; i++)
      {
         double[][] efficientJacobian = closedFormJacobianWaist.getUpdatedTransform(roll[i], pitch[i]);
         double[][] matlabJacobian = new double[2][2];

         matlabJacobian[0][0] = -m12_matlab_waist[i];
         matlabJacobian[0][1] = -m22_matlab_waist[i];
         matlabJacobian[1][0] = -m11_matlab_waist[i];
         matlabJacobian[1][1] = -m21_matlab_waist[i];

         compareMatrices(efficientJacobian, matlabJacobian, TOLERANCE);
      }
   }

   @Test
   public void testEfficientMatchesInefficientJacobianAnkle()
   {
      InefficientPushrodTransmissionJacobian inefficientButReadablePushrodTransmission = new InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint.ANKLE, null, null);

      for (int i = 0; i < 7; i++)
      {
         double[][] efficientJacobian = closedFormJacobianAnkle.getUpdatedTransform(roll[i], pitch[i]);
         double[][] inefficientJacobian = new double[2][2];
         double[][] modifiedInefficientJacobian = new double[2][2];

         inefficientButReadablePushrodTransmission.computeJacobian(inefficientJacobian, pitch[i], roll[i]);
         modifiedInefficientJacobian[0][0] = inefficientJacobian[0][1];
         modifiedInefficientJacobian[0][1] = inefficientJacobian[1][1];
         modifiedInefficientJacobian[1][0] = inefficientJacobian[0][0];
         modifiedInefficientJacobian[1][1] = inefficientJacobian[1][0];
         compareMatrices(efficientJacobian, modifiedInefficientJacobian, TOLERANCE);
      }
   }

   // The following test is just for achieving proper renishaw jacobian matrix signs/element indices. It should never be used in Bamboo.
   @Disabled
   @Test
   public void testEfficientKindaMatchesInefficientJacobianAnkle()
   {
      closedFormJacobianAnkleRenishaws.useFuteks(false);

      InefficientPushrodTransmissionJacobian inefficientButReadablePushrodTransmission = new InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint.ANKLE, null, null);

      for (int i = 0; i < 7; i++)
      {
         double[][] efficientJacobian = closedFormJacobianAnkleRenishaws.getUpdatedTransform(roll[i], pitch[i]);
         double[][] inefficientJacobian = new double[2][2];
         double[][] modifiedInefficientJacobian = new double[2][2];

         inefficientButReadablePushrodTransmission.computeJacobian(inefficientJacobian, pitch[i], roll[i]);
         modifiedInefficientJacobian[0][0] = inefficientJacobian[0][1];
         modifiedInefficientJacobian[0][1] = inefficientJacobian[1][1];
         modifiedInefficientJacobian[1][0] = inefficientJacobian[0][0];
         modifiedInefficientJacobian[1][1] = inefficientJacobian[1][0];
         compareMatrices(efficientJacobian, modifiedInefficientJacobian, TOLERANCE_GOOD_ENOUGH_FOR_GOVERNMENT_WORK);
      }
   }

	@Disabled
   @Test
   public void testEfficientMatchesInefficientJacobianWaist()
   {
      InefficientPushrodTransmissionJacobian inefficientButReadablePushrodTransmission = new InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint.WAIST, null, null);
      closedFormJacobianWaistRenishaws.useFuteks(true);
      for (int i = 0; i < 7; i++)
      {
         double[][] efficientJacobian = closedFormJacobianWaistRenishaws.getUpdatedTransform(roll[i], pitch[i]);
         double[][] inefficientJacobian = new double[2][2];
         double[][] modifiedInefficientJacobian = new double[2][2];

         inefficientButReadablePushrodTransmission.computeJacobian(inefficientJacobian, roll[i], pitch[i]);
         modifiedInefficientJacobian[0][0] = inefficientJacobian[1][0];
         modifiedInefficientJacobian[0][1] = inefficientJacobian[0][0];
         modifiedInefficientJacobian[1][0] = inefficientJacobian[1][1];
         modifiedInefficientJacobian[1][1] = inefficientJacobian[0][1];
         compareMatrices(efficientJacobian, modifiedInefficientJacobian, TOLERANCE_GOOD_ENOUGH_FOR_GOVERNMENT_WORK);
      }
   }

   @Test
   public void cosineTestAnkles()
   {
      //      A Test to ensure Renishaw and Futek Jacobians are in agreement with each other
      double rollTorque = 25;
      double pitchTorque = 300;
      double cosine5, cosine6, futek5, futek6, renishaw5, renishaw6;
      double[][] renishawJacobian, futekJacobian;

      for (int i = 0; i < 7; i++)
      {
         closedFormJacobianAnkleRenishaws.useFuteks(false);
         futekJacobian = invertMatrix(transposeMatrix(closedFormJacobianAnkle.getUpdatedTransform(roll[i], pitch[i])));

         // The cosineOfTheta functions must be called after getUpdatedTransform to update properly
         cosine5 = closedFormJacobianAnkle.cosineOfTheta5();
         cosine6 = closedFormJacobianAnkle.cosineOfTheta6();
         renishawJacobian = invertMatrix(transposeMatrix(closedFormJacobianAnkleRenishaws.getUpdatedTransform(roll[i], pitch[i])));
         futek5 = futekJacobian[0][0] * pitchTorque + futekJacobian[0][1] * rollTorque;
         futek6 = futekJacobian[1][0] * pitchTorque + futekJacobian[1][1] * rollTorque;
         renishaw5 = renishawJacobian[0][0] * pitchTorque + renishawJacobian[0][1] * rollTorque;
         renishaw6 = renishawJacobian[1][0] * pitchTorque + renishawJacobian[1][1] * rollTorque;

         if (DEBUG)
         {
            System.out.println("cos5: " + cosine6 + ", r5/f5: " + renishaw5 / futek5);
            System.out.println("cos6: " + cosine5 + ", r6/f6: " + renishaw6 / futek6);
            System.out.println(" ");
         }

         // TODO: The cosine5 and consine6 switch is a hack. Investigate this further.
         assertEquals(cosine6, renishaw5 / futek5, TOLERANCE);
         assertEquals(cosine5, renishaw6 / futek6, TOLERANCE);
      }
   }

   @Test
   public void consineTestWaist()
   {
      //      A Test to ensure Renishaw and Futek Jacobians are in agreement with each other
      double rollTorque = 25;
      double pitchTorque = 300;
      double cosine5, cosine6, futek5, futek6, renishaw5, renishaw6;
      double[][] renishawJacobian, futekJacobian;

      for (int i = 0; i < 7; i++)
      {
         closedFormJacobianWaistRenishaws.useFuteks(false);
         futekJacobian = invertMatrix(transposeMatrix(closedFormJacobianWaist.getUpdatedTransform(roll[i], pitch[i])));

         // The cosineOfTheta functions must be called after getUpdatedTransform to update properly
         cosine5 = closedFormJacobianWaist.cosineOfTheta5();
         cosine6 = closedFormJacobianWaist.cosineOfTheta6();
         renishawJacobian = invertMatrix(transposeMatrix(closedFormJacobianWaistRenishaws.getUpdatedTransform(roll[i], pitch[i])));
         futek5 = futekJacobian[0][0] * pitchTorque + futekJacobian[0][1] * rollTorque;
         futek6 = futekJacobian[1][0] * pitchTorque + futekJacobian[1][1] * rollTorque;

         renishaw5 = renishawJacobian[0][0] * pitchTorque + renishawJacobian[0][1] * rollTorque;
         renishaw6 = renishawJacobian[1][0] * pitchTorque + renishawJacobian[1][1] * rollTorque;

         if (DEBUG)
         {
            System.out.println("cos5: " + cosine5 + ", r5/f5: " + renishaw5 / futek5);
            System.out.println("cos6: " + cosine6 + ", r6/f6: " + renishaw6 / futek6);
            System.out.println(" ");
         }

         // TODO: The cosine5 and consine6 switch is a hack. Investigate this further.
         assertEquals(cosine5, renishaw5 / futek5, TOLERANCE);
         assertEquals(cosine6, renishaw6 / futek6, TOLERANCE);
      }
   }

   private void compareMatrices(double[][] matrixA, double[][] matrixB, double tolerance)
   {
      if (DEBUG)
      {
         System.out.println("matrixA: " + matrixA[0][0] + ", " + matrixA[0][1] + ", " + matrixA[1][0] + ", " + matrixA[1][1]);
         System.out.println("matrixB: " + matrixB[0][0] + ", " + matrixB[0][1] + ", " + matrixB[1][0] + ", " + matrixB[1][1]);
         System.out.println("");
      }

      assertEquals(matrixA[0][0], matrixB[0][0], tolerance);
      assertEquals(matrixA[0][1], matrixB[0][1], tolerance);
      assertEquals(matrixA[1][0], matrixB[1][0], tolerance);
      assertEquals(matrixA[1][1], matrixB[1][1], tolerance);
   }

   private double[][] invertMatrix(double[][] matrix)
   {
      double det = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

      if (det != 0.0)
      {
         double[][] inverseToReturn = new double[2][2];

         inverseToReturn[0][0] = (1.0 / det) * matrix[1][1];
         inverseToReturn[0][1] = (1.0 / det) * (-matrix[0][1]);
         inverseToReturn[1][0] = (1.0 / det) * (-matrix[1][0]);
         inverseToReturn[1][1] = (1.0 / det) * matrix[0][0];

         return inverseToReturn;
      }
      else
      {
         return null;
      }
   }

   private double[][] transposeMatrix(double[][] matrix)
   {
      double[][] transposeToReturn = new double[2][2];

      transposeToReturn[0][0] = matrix[0][0];
      transposeToReturn[0][1] = matrix[1][0];
      transposeToReturn[1][0] = matrix[0][1];
      transposeToReturn[1][1] = matrix[1][1];

      return transposeToReturn;
   }
}
