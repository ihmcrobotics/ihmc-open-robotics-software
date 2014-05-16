package us.ihmc.valkyrie.kinematics.util;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;

import us.ihmc.valkyrie.kinematics.transmissions.PushRodTransmissionJoint;

public class ClosedFormJacobian
{
   // geometric parameters from CAD
   private double h;
   private double length;
   private double lengthSquared;
   
   private DenseMatrix64F b5;
   private DenseMatrix64F b6;
   private DenseMatrix64F rod5;
   private DenseMatrix64F rod6;
   
   private final boolean DEBUG = false;

   private final DenseMatrix64F dHTM_0_1 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F dHTM_1_2 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F dHTM_0_2 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F dHTM_2_0 = SimpleMatrix.identity(4).getMatrix();
   
   private final DenseMatrix64F b5BF = new DenseMatrix64F(4, 1);    // position of futek link base 5 in bone frame (m)
   private final DenseMatrix64F b6BF = new DenseMatrix64F(4, 1);    // position of futek link base 5 in bone frame (m)
   
   private final DenseMatrix64F t5FF = new DenseMatrix64F(4, 1);
   private final DenseMatrix64F t6FF = new DenseMatrix64F(4, 1);
   
   private final Vector3d m11v = new Vector3d();
   private final Vector3d m12v = new Vector3d();
   private final Vector3d m21v = new Vector3d();
   private final Vector3d m22v = new Vector3d();
   
   private final Vector3d radius5r = new Vector3d();
   private final Vector3d radius6r = new Vector3d();
   private final Vector3d unitForceVector5r = new Vector3d();
   private final Vector3d unitForceVector6r = new Vector3d();
   
   private final Vector3d radius5p = new Vector3d();
   private final Vector3d radius6p = new Vector3d();
   private final Vector3d unitForceVector5p = new Vector3d();
   private final Vector3d unitForceVector6p = new Vector3d();
  
   public ClosedFormJacobian(PushRodTransmissionJoint pushRodTransmissionJoint)
   {
      switch (pushRodTransmissionJoint)
      {
      case ANKLE:
      {
         setupForAnkleActuators();
         break;
      }
      case WAIST:
      {
         setupForWaistActuators();
         break;
      }
      }
   }
      
   public void setupForAnkleActuators()
   {
      h = 0.0127;
      length = 0.1049655;
      lengthSquared = length * length;

      double[] b5vals = new double[] {0.0176, 0.0355, -0.0364, 1.0};
      double[] b6vals = new double[] {0.0176, -0.0355, -0.0364, 1.0};
      double[] rod5vals = new double[] {0.0, -0.0215689, -0.04128855, 1.0};
      double[] rod6vals = new double[] {0.0, -0.0215689, 0.04128855, 1.0};

      b5 = new DenseMatrix64F(4, 1, false, b5vals);
      b6 = new DenseMatrix64F(4, 1, false, b6vals);
      rod5 = new DenseMatrix64F(4, 1, false, rod5vals);
      rod6 = new DenseMatrix64F(4, 1, false, rod6vals);     
   }


   public void setupForWaistActuators()
   {
      // TODO: Add Waist parameters. Right now they are ankle parameters.
      h = 0.0127;
      length = 0.1049655;
      lengthSquared = length * length;

      double[] b5vals = new double[] {0.0176, 0.0355, -0.0364, 1.0};
      double[] b6vals = new double[] {0.0176, -0.0355, -0.0364, 1.0};
      double[] rod5vals = new double[] {0.0, -0.0215689, -0.04128855, 1.0};
      double[] rod6vals = new double[] {0.0, -0.0215689, 0.04128855, 1.0};

      b5 = new DenseMatrix64F(4, 1, false, b5vals);
      b6 = new DenseMatrix64F(4, 1, false, b6vals);
      rod5 = new DenseMatrix64F(4, 1, false, rod5vals);
      rod6 = new DenseMatrix64F(4, 1, false, rod6vals);          
   }

      
   private void denavitHartenbergTransformationMatrix(DenseMatrix64F matrixToPack, double a, double alpha, double d, double theta)
   {
      double cosTheta = Math.cos(theta);
      double sinTheta = Math.sin(theta);
      double cosAlpha = Math.cos(alpha);
      double sinAlpha = Math.sin(alpha);
      
      matrixToPack.set(0, 0, cosTheta);
      matrixToPack.set(0, 1, -sinTheta * cosAlpha);
      matrixToPack.set(0, 2, sinTheta * sinAlpha);
      matrixToPack.set(0, 3, a * cosTheta);

      matrixToPack.set(1, 0, sinTheta);
      matrixToPack.set(1, 1, cosTheta * cosAlpha);
      matrixToPack.set(1, 2, -cosTheta * sinAlpha);
      matrixToPack.set(1, 3, a * sinTheta);

      matrixToPack.set(2, 0, 0.0);
      matrixToPack.set(2, 1, sinAlpha);
      matrixToPack.set(2, 2, cosAlpha);
      matrixToPack.set(2, 3, d);

      matrixToPack.set(3, 0, 0.0);
      matrixToPack.set(3, 1, 0.0);
      matrixToPack.set(3, 2, 0.0);
      matrixToPack.set(3, 3, 1.0);
   }


   public double[][] getUpdatedTransform(double roll, double pitch)
   {
//    System.out.println( roll + " " + pitch );
      // solve for rod end heights in bone frame
      pitch = pitch;    // - 7.5*Math.PI/180;

      denavitHartenbergTransformationMatrix(dHTM_0_1, h, Math.PI / 2.0, 0.0, roll);
      denavitHartenbergTransformationMatrix(dHTM_1_2, 0.0, 0.0, 0.0, pitch);
      
      CommonOps.mult(dHTM_0_1, dHTM_1_2, dHTM_0_2);
      CommonOps.invert(dHTM_0_2, dHTM_2_0);
      CommonOps.mult(dHTM_2_0, b5, b5BF);
      CommonOps.mult(dHTM_2_0, b6, b6BF);

      printMatrixIfDebug("b5BF", b5BF);
      printMatrixIfDebug("b6BF", b6BF);

      DenseMatrix64F t5 = rod5;
      DenseMatrix64F t6 = rod6;
      double pushrodHeightSolution5 = Math.sqrt(lengthSquared - Math.pow(t5.get(1) - b5BF.get(1), 2) - Math.pow(t5.get(2) - b5BF.get(2), 2))
                                      + b5BF.get(0);
      t5.set(0, 0, pushrodHeightSolution5);

      double pushrodHeightSolution6 = Math.sqrt(lengthSquared - Math.pow(t6.get(1) - b6BF.get(1), 2) - Math.pow(t6.get(2) - b6BF.get(2), 2))
                                      + b6BF.get(0);
      t6.set(0, 0, pushrodHeightSolution6);

      printIfDebug("pushrodHeightSolution5 = " + pushrodHeightSolution5);
      printIfDebug("pushrodHeightSolution6 = " + pushrodHeightSolution6);

      // convert position of tops of futek links to foot frame
      
      CommonOps.mult(dHTM_0_2, t5, t5FF);
      CommonOps.mult(dHTM_0_2, t6, t6FF);

      printMatrixIfDebug("t5FF", t5FF);
      printMatrixIfDebug("t6FF", t6FF);

      // solve for torques on roll axis in foot frame
      radius5r.set(b5.get(0), b5.get(1), 0.0);
      radius6r.set(b6.get(0), b6.get(1), 0.0);
      unitForceVector5r.set(t5FF.get(0) - b5.get(0), t5FF.get(1) - b5.get(1), t5FF.get(2) - b5.get(2));
      unitForceVector6r.set(t6FF.get(0) - b6.get(0), t6FF.get(1) - b6.get(1), t6FF.get(2) - b6.get(2));
      
      m11v.cross(radius5r, unitForceVector5r);
      m12v.cross(radius6r, unitForceVector6r);
      double m11 = m11v.getZ() / length;
      double m12 = m12v.getZ() / length;

      // solve for torques on pitch axis in bone frame
      radius5p.set(b5BF.get(0), b5BF.get(1), 0.0);
      radius6p.set(b6BF.get(0), b6BF.get(1), 0.0);
      unitForceVector5p.set(t5.get(0) - b5BF.get(0), t5.get(1) - b5BF.get(1), t5.get(2) - b5BF.get(2));
      unitForceVector6p.set(t6.get(0) - b6BF.get(0), t6.get(1) - b6BF.get(1), t6.get(2) - b6BF.get(2));
     
      m21v.cross(radius5p, unitForceVector5p);
      m22v.cross(radius6p, unitForceVector6p);
      double m21 = m21v.getZ() / length;
      double m22 = m22v.getZ() / length;

//    System.out.println("J: "+m11 +", "+ m12 +", "+ m21 +", "+ m22);
      double[][] output =
      {
         {-m11, -m12}, {-m21, -m22}
      };

      return output;
   }

   private void printMatrixIfDebug(String name, DenseMatrix64F matrix)
   {
      if (DEBUG)
      {
         System.out.println(name + "= ");

         int numCols = matrix.getNumCols();
         int numRows = matrix.getNumRows();

         for (int i = 0; i < numRows; i++)
         {
            for (int j = 0; j < numCols; j++)
            {
               System.out.print(matrix.get(i, j));
               if (j < numCols - 1)
                  System.out.print(", ");
            }

            System.out.println();
         }
      }

   }


   private void printIfDebug(String message)
   {
      if (DEBUG)
         System.out.println(message);
   }
}
