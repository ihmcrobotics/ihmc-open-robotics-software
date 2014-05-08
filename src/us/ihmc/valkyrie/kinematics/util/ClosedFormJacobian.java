package us.ihmc.valkyrie.kinematics.util;

import javax.vecmath.Vector3d;

import org.ejml.*;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;

public class ClosedFormJacobian
{
   // geometric parameters from CAD
   private static final double h = 0.0127;
   private static final double length = 0.1049655;
   private static final double []b5vals = new double[]{ 0.0176,  0.0355, -0.0364, 1.0 };
   private static final double []b6vals = new double[]{ 0.0176, - 0.0355, -0.0364, 1.0 };
   private static final double []rod5vals = new double[]{ 0, -0.0215689, -0.04128855, 1.0 };
   private static final double []rod6vals = new double[]{ 0, -0.0215689,  0.04128855, 1.0 };
   
   private static final DenseMatrix64F b5 = new DenseMatrix64F( 4, 1, false, b5vals );
   private static final DenseMatrix64F b6 = new DenseMatrix64F( 4, 1, false, b6vals );
   private static final DenseMatrix64F rod5 = new DenseMatrix64F( 4, 1, false, rod5vals );
   private static final DenseMatrix64F rod6 = new DenseMatrix64F( 4, 1, false, rod6vals );
   
   @SuppressWarnings("unused")
   private boolean returnInverseJ;
   
   private DenseMatrix64F dHTM( double a, double alpha, double d, double theta ){
      DenseMatrix64F transformationMatrix = SimpleMatrix.identity(4).getMatrix();
      transformationMatrix.set(0,0,Math.cos(theta));
      transformationMatrix.set(1,0,Math.sin(theta));
      transformationMatrix.set(0,1,-Math.sin(theta)*Math.cos(alpha));
      transformationMatrix.set(1,1,Math.cos(theta)*Math.cos(alpha));
      transformationMatrix.set(2,1,Math.sin(alpha));
      transformationMatrix.set(0,2,Math.sin(theta)*Math.sin(alpha));
      transformationMatrix.set(1,2,-Math.cos(theta)*Math.sin(alpha));
      transformationMatrix.set(2,2,Math.cos(alpha));
      transformationMatrix.set(0,3,a*Math.cos(theta));
      transformationMatrix.set(1,3,a*Math.sin(theta));
      transformationMatrix.set(2,3,d);
      return transformationMatrix;
   }
   

   public double[][] getUpdatedTransform( double roll, double pitch ){
      System.out.println( roll + " " + pitch );
      // solve for rod end heights in bone frame
      pitch =  pitch - 7.5*Math.PI/180;
      DenseMatrix64F dHTM_0_1 = dHTM( h, Math.PI/2.0, 0.0, roll );
      DenseMatrix64F dHTM_1_2 = dHTM( 0.0, 0.0, 0.0, pitch );
      DenseMatrix64F dHTM_0_2 = SimpleMatrix.identity(4).getMatrix();
      DenseMatrix64F dHTM_2_0 = SimpleMatrix.identity(4).getMatrix();
      DenseMatrix64F b5BF = new DenseMatrix64F( 4, 1 ); // position of futek link base 5 in bone frame (m)
      DenseMatrix64F b6BF = new DenseMatrix64F( 4, 1 ); // position of futek link base 5 in bone frame (m)
      CommonOps.mult(dHTM_0_1,dHTM_1_2,dHTM_0_2);
      CommonOps.invert(dHTM_0_2,dHTM_2_0);
      CommonOps.mult(dHTM_2_0,b5,b5BF);
      CommonOps.mult(dHTM_2_0,b6,b6BF);
      DenseMatrix64F t5 = rod5;     
      DenseMatrix64F t6 = rod6;     
      t5.set(0,0,Math.sqrt(Math.pow(length, 2) - Math.pow(t5.get(1) - b5BF.get(1),2) - Math.pow(t5.get(2)-b5BF.get(2),2) + b5BF.get(0)));
      t6.set(0,0,Math.sqrt(Math.pow(length, 2) - Math.pow(t6.get(1) - b6BF.get(1),2) - Math.pow(t6.get(2)-b6BF.get(2),2) + b6BF.get(0)));
      
      // convert position of tops of futek links to foot frame
      DenseMatrix64F t5FF = new DenseMatrix64F(4, 1);
      DenseMatrix64F t6FF = new DenseMatrix64F(4, 1);
      CommonOps.mult(dHTM_0_2, t5, t5FF);
      CommonOps.mult(dHTM_0_2, t6, t6FF);
      
      // solve for torques on roll axis in foot frame
      Vector3d radius5r = new Vector3d( b5.get(0), b5.get(1), 0.0 );
      Vector3d radius6r = new Vector3d( b6.get(0), b6.get(1), 0.0 );
      Vector3d unitForceVector5r = new Vector3d(t5FF.get(0)-b5.get(0), t5FF.get(1)-b5.get(1), 0.0);
      Vector3d unitForceVector6r = new Vector3d(t6FF.get(0)-b6.get(0), t6FF.get(1)-b6.get(1), 0.0);
      Vector3d m11v = new Vector3d(); m11v.cross(radius5r, unitForceVector5r); 
      Vector3d m12v = new Vector3d(); m12v.cross(radius5r, unitForceVector5r);
      double m11 = m11v.getZ()/length;
      double m12 = m12v.getZ()/length;
      
      // solve for torques on pitch axis in bone frame
      Vector3d radius5p = new Vector3d( b5BF.get(0), b5BF.get(1), 0.0 );
      Vector3d radius6p = new Vector3d( b6BF.get(0), b6BF.get(1), 0.0 );
      Vector3d unitForceVector5p = new Vector3d( t5.get(0)-b5BF.get(0), t5.get(1)-b5BF.get(1), 0.0 );
      Vector3d unitForceVector6p = new Vector3d( t6.get(0)-b6BF.get(0), t6.get(1)-b6BF.get(1), 0.0 );
      Vector3d m21v = new Vector3d(); m21v.cross(radius5p, unitForceVector5p); 
      Vector3d m22v = new Vector3d(); m22v.cross(radius5p, unitForceVector5p);
      double m21 = m21v.getZ()/length;
      double m22 = m22v.getZ()/length;
//      System.out.println("J: "+m11 +", "+ m12 +", "+ m21 +", "+ m22);
      double[][] output = {{m11, m12},{m21, m22}};
      return output;
   } 
}
