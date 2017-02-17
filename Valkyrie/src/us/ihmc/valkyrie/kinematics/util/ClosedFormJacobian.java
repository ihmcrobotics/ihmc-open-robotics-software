package us.ihmc.valkyrie.kinematics.util;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.valkyrie.kinematics.transmissions.PushRodTransmissionJoint;

public class ClosedFormJacobian
{
	//Will wuz here
   private double h;
   private double length;
   private double lengthSquared;
   
   private final boolean DEBUG = false;
   
   private boolean useFuteks = true;

   private final DenseMatrix64F transform_0_1 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F transform_1_2 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F transform_0_2 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F transform_2_0 = SimpleMatrix.identity(4).getMatrix();
   private final DenseMatrix64F transform_1_0 = SimpleMatrix.identity(4).getMatrix();
   
   private final Vector3D m11v = new Vector3D();
   private final Vector3D m12v = new Vector3D();
   private final Vector3D m21v = new Vector3D();
   private final Vector3D m22v = new Vector3D();
   
   private final Vector3D radius5UpperDOF = new Vector3D();
   private final Vector3D radius6UpperDOF = new Vector3D();
   private final Vector3D unitForceVector5UpperDOF = new Vector3D();
   private final Vector3D unitForceVector6UpperDOF = new Vector3D();
   
   private final Vector3D radius5LowerDOF = new Vector3D();
   private final Vector3D radius6LowerDOF = new Vector3D();
   private final Vector3D unitForceVector5LowerDOF = new Vector3D();
   private final Vector3D unitForceVector6LowerDOF = new Vector3D();
   
   private PushRodTransmissionJoint pushRodTransmissionJoint;
   
   private RowD1Matrix64F t5ZerothFrame = new DenseMatrix64F(4, 1);
   private RowD1Matrix64F t6ZerothFrame = new DenseMatrix64F(4, 1);
   private DenseMatrix64F b5ZerothFrame = new DenseMatrix64F(4, 1);
   private DenseMatrix64F b6ZerothFrame = new DenseMatrix64F(4, 1);
   
   private RowD1Matrix64F t5FirstFrame = new DenseMatrix64F(4, 1);
   private RowD1Matrix64F t6FirstFrame = new DenseMatrix64F(4, 1);
   private RowD1Matrix64F b5FirstFrame = new DenseMatrix64F(4, 1);
   private RowD1Matrix64F b6FirstFrame = new DenseMatrix64F(4, 1);
   
   private DenseMatrix64F t5SecondFrame = new DenseMatrix64F(4, 1);
   private DenseMatrix64F t6SecondFrame = new DenseMatrix64F(4, 1);
   private RowD1Matrix64F b5SecondFrame = new DenseMatrix64F(4, 1);
   private RowD1Matrix64F b6SecondFrame = new DenseMatrix64F(4, 1);
   
//   private RowD1Matrix64F renishawUnitForceVectorZerothFrame = new DenseMatrix64F(4, 1);
//   private RowD1Matrix64F renishawUnitForceVectorFirstFrame = new DenseMatrix64F(4, 1);
//   private final DenseMatrix64F renishawUnitForceVectorSecondFrame = new DenseMatrix64F(4, 1, false, 0.0, 0.0, -1.0, 1.0);
   
   private DenseMatrix64F transform01StaticPart  = new DenseMatrix64F(4, 4);
   private DenseMatrix64F transform12StaticPart  = new DenseMatrix64F(4, 4);
   
   private DenseMatrix64F pitchTransform = new DenseMatrix64F(4, 4);
   private DenseMatrix64F rollTransform  = new DenseMatrix64F(4, 4);
     
   public ClosedFormJacobian(PushRodTransmissionJoint pushRodTransmissionJoint)
   {
      this.pushRodTransmissionJoint = pushRodTransmissionJoint;
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
      default:
         break;
      }
   }
      
   public void setupForAnkleActuators()
   {
      h = 0.0127;
      length = 0.1049655;
      lengthSquared = length * length;

      b5ZerothFrame = new DenseMatrix64F(4, 1, false, -0.0364, -0.0355, 0.0176, 1.0);
      b6ZerothFrame = new DenseMatrix64F(4, 1, false, -0.0364,  0.0355, 0.0176, 1.0);
      
      t5SecondFrame = new DenseMatrix64F(4, 1, false, -0.0215689, -0.04128855, 0, 1.0);
      t6SecondFrame = new DenseMatrix64F(4, 1, false, -0.0215689,  0.04128855, 0, 1.0);  
      
      transform01StaticPart = new DenseMatrix64F(4,4,true, 1.0, 0.0 ,0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, h, 0.0, 0.0, 0.0, 1.0);
      transform12StaticPart = new DenseMatrix64F(4,4,true, 1.0, 0.0 ,0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
   }


   public void setupForWaistActuators()
   {
      h = 0.02032;
      length = 0.1310005;
      lengthSquared = length * length;
      
      b5ZerothFrame = new DenseMatrix64F(4, 1, false, -0.0762,  0.0508, 0.0, 1.0);
      b6ZerothFrame = new DenseMatrix64F(4, 1, false, -0.0762, -0.0508, 0.0, 1.0);
      
      t5SecondFrame = new DenseMatrix64F(4, 1, false, 0.0,  0.06985123, 0.0, 1.0);
      t6SecondFrame = new DenseMatrix64F(4, 1, false, 0.0, -0.06985123, 0.0, 1.0);
      
      transform01StaticPart = new DenseMatrix64F(4,4,true, 1.0, 0.0 ,0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, h, 0.0, 0.0, 0.0, 1.0);
      transform12StaticPart = new DenseMatrix64F(4,4,true, 0.913545457642601, 0.0, 0.406736643075800, -0.00598410000000000, 0.0, 1.0, 0.0, 0.0, -0.406736643075800, 0.0, 0.913545457642601, 0.0886199400000000, 0.0, 0.0, 0.0, 1.0);
   }
   
   private void rotX(DenseMatrix64F matrixToPack, double alpha){
      
      double cosAlpha = Math.cos(alpha);
      double sinAlpha = Math.sin(alpha);
      
      matrixToPack.set(0, 0, 1.0);
      matrixToPack.set(0, 1, 0.0);
      matrixToPack.set(0, 2, 0.0);
      matrixToPack.set(0, 3, 0.0);

      matrixToPack.set(1, 0, 0.0);
      matrixToPack.set(1, 1, cosAlpha);
      matrixToPack.set(1, 2, -sinAlpha);
      matrixToPack.set(1, 3, 0.0);

      matrixToPack.set(2, 0, 0.0);
      matrixToPack.set(2, 1, sinAlpha);
      matrixToPack.set(2, 2, cosAlpha);
      matrixToPack.set(2, 3, 0.0);

      matrixToPack.set(3, 0, 0.0);
      matrixToPack.set(3, 1, 0.0);
      matrixToPack.set(3, 2, 0.0);
      matrixToPack.set(3, 3, 1.0);
   }
   
   private void rotY(DenseMatrix64F matrixToPack, double beta){
      
      double cosBeta = Math.cos(beta);
      double sinBeta = Math.sin(beta);
      
      matrixToPack.set(0, 0, cosBeta);
      matrixToPack.set(0, 1, 0.0);
      matrixToPack.set(0, 2, sinBeta);
      matrixToPack.set(0, 3, 0.0);

      matrixToPack.set(1, 0, 0.0);
      matrixToPack.set(1, 1, 1.0);
      matrixToPack.set(1, 2, 0.0);
      matrixToPack.set(1, 3, 0.0);

      matrixToPack.set(2, 0, -sinBeta);
      matrixToPack.set(2, 1, 0.0);
      matrixToPack.set(2, 2, cosBeta);
      matrixToPack.set(2, 3, 0.0);

      matrixToPack.set(3, 0, 0.0);
      matrixToPack.set(3, 1, 0.0);
      matrixToPack.set(3, 2, 0.0);
      matrixToPack.set(3, 3, 1.0);
   }
   
//   private void denavitHartenbergTransformationMatrix(DenseMatrix64F matrixToPack, double a, double alpha, double d, double theta)
//   {
//      double cosTheta = Math.cos(theta);
//      double sinTheta = Math.sin(theta);
//      double cosAlpha = Math.cos(alpha);
//      double sinAlpha = Math.sin(alpha);
//      
//      matrixToPack.set(0, 0, cosTheta);
//      matrixToPack.set(0, 1, -sinTheta * cosAlpha);
//      matrixToPack.set(0, 2, sinTheta * sinAlpha);
//      matrixToPack.set(0, 3, a * cosTheta);
//
//      matrixToPack.set(1, 0, sinTheta);
//      matrixToPack.set(1, 1, cosTheta * cosAlpha);
//      matrixToPack.set(1, 2, -cosTheta * sinAlpha);
//      matrixToPack.set(1, 3, a * sinTheta);
//
//      matrixToPack.set(2, 0, 0.0);
//      matrixToPack.set(2, 1, sinAlpha);
//      matrixToPack.set(2, 2, cosAlpha);
//      matrixToPack.set(2, 3, d);
//
//      matrixToPack.set(3, 0, 0.0);
//      matrixToPack.set(3, 1, 0.0);
//      matrixToPack.set(3, 2, 0.0);
//      matrixToPack.set(3, 3, 1.0);
//   }


   public double[][] getUpdatedTransform( double roll, double pitch )
   {
      double[][] output =  new double[2][2];
      switch (pushRodTransmissionJoint)
      {
      case ANKLE:
      { 
      //Ankle Specific
      pitch = -pitch;
//      roll = -roll;
      rotX(rollTransform, roll);
      CommonOps.mult(rollTransform, transform01StaticPart, transform_0_1);
      rotY(pitchTransform, pitch);
      CommonOps.mult(pitchTransform, transform12StaticPart, transform_1_2);
      //End Ankle Specific
      
      CommonOps.mult(transform_0_1, transform_1_2, transform_0_2);
      CommonOps.invert(transform_0_2, transform_2_0);
      CommonOps.invert(transform_0_1, transform_1_0);
      
      //Solve for pushrod heights (z in second frame)
      CommonOps.mult(transform_2_0, b5ZerothFrame, b5SecondFrame);
      CommonOps.mult(transform_2_0, b6ZerothFrame, b6SecondFrame);
      
      double pushrodHeightSolution5=Math.sqrt(lengthSquared - Math.pow(t5SecondFrame.get(0)-b5SecondFrame.get(0), 2) - Math.pow(t5SecondFrame.get(1)-b5SecondFrame.get(1), 2)) + b5SecondFrame.get(2);
      t5SecondFrame.set(2, 0, pushrodHeightSolution5);
      double pushrodHeightSolution6=Math.sqrt(lengthSquared - Math.pow(t6SecondFrame.get(0)-b6SecondFrame.get(0), 2) - Math.pow(t6SecondFrame.get(1)-b6SecondFrame.get(1), 2)) + b6SecondFrame.get(2);
      t6SecondFrame.set(2, 0, pushrodHeightSolution6);
      
      //Convert tops positions to zeroth frame
      CommonOps.mult(transform_0_2, t5SecondFrame, t5ZerothFrame);
      CommonOps.mult(transform_0_2, t6SecondFrame, t6ZerothFrame);
      
      //Convert tops to first joint frame
      CommonOps.mult(transform_1_2, t5SecondFrame, t5FirstFrame);
      CommonOps.mult(transform_1_2, t6SecondFrame, t6FirstFrame);
      
      if(useFuteks){
           
         //Convert bottoms to first joint frame
         CommonOps.mult(transform_1_0, b5ZerothFrame, b5FirstFrame);
         CommonOps.mult(transform_1_0, b6ZerothFrame, b6FirstFrame);
         
         //Compute radius and Force vectors for use in r x F calculation in zeroth frame for futeks
         radius5LowerDOF.set(b5ZerothFrame.get(0), b5ZerothFrame.get(1), b5ZerothFrame.get(2));
         radius6LowerDOF.set(b6ZerothFrame.get(0), b6ZerothFrame.get(1), b6ZerothFrame.get(2));
         unitForceVector5LowerDOF.set(t5ZerothFrame.get(0) - b5ZerothFrame.get(0), t5ZerothFrame.get(1) - b5ZerothFrame.get(1), t5ZerothFrame.get(2) - b5ZerothFrame.get(2));
         unitForceVector6LowerDOF.set(t6ZerothFrame.get(0) - b6ZerothFrame.get(0), t6ZerothFrame.get(1) - b6ZerothFrame.get(1), t6ZerothFrame.get(2) - b6ZerothFrame.get(2));
         
         //Compute radius and Force vectors for use in r x F calculation in first joint frame for futeks
         radius5UpperDOF.set(b5FirstFrame.get(0), b5FirstFrame.get(1), b5FirstFrame.get(2));
         radius6UpperDOF.set(b6FirstFrame.get(0), b6FirstFrame.get(1), b6FirstFrame.get(2));
         unitForceVector5UpperDOF.set(t5FirstFrame.get(0) - b5FirstFrame.get(0), t5FirstFrame.get(1) - b5FirstFrame.get(1), t5FirstFrame.get(2) - b5FirstFrame.get(2));
         unitForceVector6UpperDOF.set(t6FirstFrame.get(0) - b6FirstFrame.get(0), t6FirstFrame.get(1) - b6FirstFrame.get(1), t6FirstFrame.get(2) - b6FirstFrame.get(2));
         
         m11v.cross(radius5LowerDOF, unitForceVector5LowerDOF);
         m12v.cross(radius6LowerDOF, unitForceVector6LowerDOF);
         m21v.cross(radius5UpperDOF, unitForceVector5UpperDOF);
         m22v.cross(radius6UpperDOF, unitForceVector6UpperDOF);
         
         double m11 = m11v.getX() / length;
         double m12 = m12v.getX() / length;
         double m21 = m21v.getY() / length;
         double m22 = m22v.getY() / length;
         
         output[1][1] =   m11; output[0][1] =   m12;
         output[1][0] =  -m21; output[0][0] =  -m22;
         
         break;
      
      }else{        
//         roll = -roll;
         double J2A_11, J2A_12, J2A_21, J2A_22;
         double cosRoll = Math.cos(roll);
         double cosPitch = Math.cos(pitch);
         double sinRoll = Math.sin(roll);
         double sinPitch = Math.sin(pitch);
         double cosPitchMinusRoll = Math.cos(pitch - roll);
         double cosPitchPlusRoll = Math.cos(pitch + roll);
         double sinPitchMinusRoll = Math.sin(pitch - roll);
         double sinPitchPlusRoll = Math.sin(pitch + roll);
         
         //To find the source of the following symbolic Jacobian expressions, search for TrueAnalyticalJacobian.m in IHMCUtilities/matlab/ValkyrieJacobiansMatlab/ANKLE.
         J2A_11 = cosPitch*cosRoll*(7.1E1/2.0E3)-cosPitch*sinRoll*(1.1E1/6.25E2)-((cosPitchMinusRoll*(-1.1E1/1.25E3)+sinPitchMinusRoll*(7.1E1/4.0E3)+cosPitchPlusRoll*(1.1E1/1.25E3)+sinPitchPlusRoll*(7.1E1/4.0E3))*(cosPitchMinusRoll*(7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)-cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17)*2.0+(cosRoll*(1.1E1/6.25E2)+sinRoll*(7.1E1/2.0E3))*(cosRoll*(-7.1E1/2.0E3)+sinRoll*(1.1E1/6.25E2)+5.950307148629379E15/1.441151880758559E17)*2.0)*1.0/Math.sqrt(-Math.pow(cosRoll*(-7.1E1/2.0E3)+sinRoll*(1.1E1/6.25E2)+5.950307148629379E15/1.441151880758559E17,2.0)-Math.pow(cosPitchMinusRoll*(7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)-cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17,2.0)+3.96956501382951E14/3.602879701896397E16)*(1.0/2.0);
         J2A_12 = cosPitchMinusRoll*(-7.1E1/4.0E3)+cosPitchPlusRoll*(7.1E1/4.0E3)-cosPitch*(9.1E1/2.5E3)+sinPitch*(1.27E2/1.0E4)-cosRoll*sinPitch*(1.1E1/6.25E2)-1.0/Math.sqrt(-Math.pow(cosRoll*(-7.1E1/2.0E3)+sinRoll*(1.1E1/6.25E2)+5.950307148629379E15/1.441151880758559E17,2.0)-Math.pow(cosPitchMinusRoll*(7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)-cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17,2.0)+3.96956501382951E14/3.602879701896397E16)*(cosPitchMinusRoll*(1.1E1/1.25E3)-sinPitchMinusRoll*(7.1E1/4.0E3)+cosPitchPlusRoll*(1.1E1/1.25E3)+sinPitchPlusRoll*(7.1E1/4.0E3)-cosPitch*(1.27E2/1.0E4)-sinPitch*(9.1E1/2.5E3))*(cosPitchMinusRoll*(7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)-cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17);
         J2A_21 = cosPitch*cosRoll*(-7.1E1/2.0E3)-cosPitch*sinRoll*(1.1E1/6.25E2)+((cosPitchMinusRoll*(1.1E1/1.25E3)+sinPitchMinusRoll*(7.1E1/4.0E3)-cosPitchPlusRoll*(1.1E1/1.25E3)+sinPitchPlusRoll*(7.1E1/4.0E3))*(cosPitchMinusRoll*(-7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)+cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17)*2.0-(cosRoll*(1.1E1/6.25E2)-sinRoll*(7.1E1/2.0E3))*(cosRoll*(7.1E1/2.0E3)+sinRoll*(1.1E1/6.25E2)-5.950307148629379E15/1.441151880758559E17)*2.0)*1.0/Math.sqrt(-Math.pow(cosRoll*(7.1E1/2.0E3)+sinRoll*(1.1E1/6.25E2)-5.950307148629379E15/1.441151880758559E17,2.0)-Math.pow(cosPitchMinusRoll*(-7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)+cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17,2.0)+3.96956501382951E14/3.602879701896397E16)*(1.0/2.0);
         J2A_22 = cosPitchMinusRoll*(7.1E1/4.0E3)-cosPitchPlusRoll*(7.1E1/4.0E3)-cosPitch*(9.1E1/2.5E3)+sinPitch*(1.27E2/1.0E4)-cosRoll*sinPitch*(1.1E1/6.25E2)-1.0/Math.sqrt(-Math.pow(cosRoll*(7.1E1/2.0E3)+sinRoll*(1.1E1/6.25E2)-5.950307148629379E15/1.441151880758559E17,2.0)-Math.pow(cosPitchMinusRoll*(-7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)+cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17,2.0)+3.96956501382951E14/3.602879701896397E16)*(cosPitchMinusRoll*(1.1E1/1.25E3)+sinPitchMinusRoll*(7.1E1/4.0E3)+cosPitchPlusRoll*(1.1E1/1.25E3)-sinPitchPlusRoll*(7.1E1/4.0E3)-cosPitch*(1.27E2/1.0E4)-sinPitch*(9.1E1/2.5E3))*(cosPitchMinusRoll*(-7.1E1/4.0E3)+sinPitchMinusRoll*(1.1E1/1.25E3)+cosPitchPlusRoll*(7.1E1/4.0E3)+sinPitchPlusRoll*(1.1E1/1.25E3)+cosPitch*(9.1E1/2.5E3)-sinPitch*(1.27E2/1.0E4)-6.216812160178655E15/2.882303761517117E17);         
         
         output[1][1]=-J2A_11; output[1][0]=J2A_12;
         output[0][1]=-J2A_21; output[0][0]=J2A_22;
         
         break;
      }
      
      }
      
      case WAIST:
      {
    	  pitch=-pitch;
    	  if(useFuteks){
        // Create the transformation matrices
        rotY(pitchTransform, -pitch);
        CommonOps.mult(pitchTransform, transform01StaticPart, transform_0_1);
        rotX(rollTransform, -roll);
        CommonOps.mult(rollTransform, transform12StaticPart, transform_1_2);
        CommonOps.mult(transform_0_1, transform_1_2, transform_0_2);
        CommonOps.invert(transform_0_2, transform_2_0);
        CommonOps.invert(transform_0_1, transform_1_0);
        
        //Solve for pushrod heights (z in second frame)
        CommonOps.mult(transform_2_0, b5ZerothFrame, b5SecondFrame);
        CommonOps.mult(transform_2_0, b6ZerothFrame, b6SecondFrame);
        
        double pushrodHeightSolution5=Math.sqrt(lengthSquared - Math.pow(t5SecondFrame.get(0)-b5SecondFrame.get(0), 2) - Math.pow(t5SecondFrame.get(1)-b5SecondFrame.get(1), 2)) + b5SecondFrame.get(2);
        t5SecondFrame.set(2, 0, pushrodHeightSolution5);
        double pushrodHeightSolution6=Math.sqrt(lengthSquared - Math.pow(t6SecondFrame.get(0)-b6SecondFrame.get(0), 2) - Math.pow(t6SecondFrame.get(1)-b6SecondFrame.get(1), 2)) + b6SecondFrame.get(2);
        t6SecondFrame.set(2, 0, pushrodHeightSolution6);
        
        //Convert tops positions to zeroth frame
        CommonOps.mult(transform_0_2, t5SecondFrame, t5ZerothFrame);
        CommonOps.mult(transform_0_2, t6SecondFrame, t6ZerothFrame);
        
        //Convert tops to first joint frame
        CommonOps.mult(transform_1_2, t5SecondFrame, t5FirstFrame);
        CommonOps.mult(transform_1_2, t6SecondFrame, t6FirstFrame);
        
           //Convert bottoms to first joint frame
           CommonOps.mult(transform_1_0, b5ZerothFrame, b5FirstFrame);
           CommonOps.mult(transform_1_0, b6ZerothFrame, b6FirstFrame);
           
           //Compute radius and Force vectors for use in r x F calculation in zeroth frame for futeks
           radius5LowerDOF.set(b5ZerothFrame.get(0), b5ZerothFrame.get(1), b5ZerothFrame.get(2));
           radius6LowerDOF.set(b6ZerothFrame.get(0), b6ZerothFrame.get(1), b6ZerothFrame.get(2));
           unitForceVector5LowerDOF.set(t5ZerothFrame.get(0) - b5ZerothFrame.get(0), t5ZerothFrame.get(1) - b5ZerothFrame.get(1), t5ZerothFrame.get(2) - b5ZerothFrame.get(2));
           unitForceVector6LowerDOF.set(t6ZerothFrame.get(0) - b6ZerothFrame.get(0), t6ZerothFrame.get(1) - b6ZerothFrame.get(1), t6ZerothFrame.get(2) - b6ZerothFrame.get(2));
           
           //Compute radius and Force vectors for use in r x F calculation in first joint frame for futeks
           radius5UpperDOF.set(b5FirstFrame.get(0), b5FirstFrame.get(1), b5FirstFrame.get(2));
           radius6UpperDOF.set(b6FirstFrame.get(0), b6FirstFrame.get(1), b6FirstFrame.get(2));
           unitForceVector5UpperDOF.set(t5FirstFrame.get(0) - b5FirstFrame.get(0), t5FirstFrame.get(1) - b5FirstFrame.get(1), t5FirstFrame.get(2) - b5FirstFrame.get(2));
           unitForceVector6UpperDOF.set(t6FirstFrame.get(0) - b6FirstFrame.get(0), t6FirstFrame.get(1) - b6FirstFrame.get(1), t6FirstFrame.get(2) - b6FirstFrame.get(2));
           
           m11v.cross(radius5UpperDOF, unitForceVector5UpperDOF);
           m12v.cross(radius6UpperDOF, unitForceVector6UpperDOF);
           m21v.cross(radius5LowerDOF, unitForceVector5LowerDOF);
           m22v.cross(radius6LowerDOF, unitForceVector6LowerDOF);
           
           double m11 = m11v.getX() / length;
           double m12 = m12v.getX() / length;
           double m21 = m21v.getY() / length;
           double m22 = m22v.getY() / length;
           
           output[0][0] = m21;  output[0][1] = -m11;
           output[1][0] = m22;  output[1][1] = -m12;
           break;  
           
        }else{
//           roll = -roll;
           double J2A_11, J2A_12, J2A_21, J2A_22;
           double cosRoll = Math.cos(roll);
           double cosPitch = Math.cos(pitch);
           double sinRoll = Math.sin(roll);
           double sinPitch = Math.sin(pitch);
           double cosPitchMinusRoll = Math.cos(pitch - roll);
           double cosPitchPlusRoll = Math.cos(pitch + roll);
           double sinPitchMinusRoll = Math.sin(pitch - roll);
           double sinPitchPlusRoll = Math.sin(pitch + roll);
           
           //To find the source of the following symbolic Jacobian search for TrueAnalyticalJacobian.m in IHMCUtilities/matlab/ValkyrieJacobiansMatlab/WAIST.
//           J2A_11=cosPitchMinusRoll*-3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2-cosRoll*4.640810924824413E-2+sinRoll*1.856324369929765E-2+((sinPitchMinusRoll*3.81E-2+sinPitchPlusRoll*3.81E-2-cosRoll*2.032E-2-sinRoll*5.08E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2)*2.0+(cosPitchMinusRoll*1.549666610118799E-2-cosPitchPlusRoll*1.549666610118799E-2+cosRoll*2.066222146825065E-2-sinRoll*8.26488858730026E-3)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0)*1.0/Math.sqrt(-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
//           J2A_12=cosPitchMinusRoll*3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2+sinPitch*3.099333220237597E-2-((cosPitchMinusRoll*1.549666610118799E-2+cosPitchPlusRoll*1.549666610118799E-2-sinPitch*6.961216387236619E-2)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0+(sinPitchMinusRoll*3.81E-2-sinPitchPlusRoll*3.81E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2)*2.0)*1.0/Math.sqrt(-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
//           J2A_21=cosPitchMinusRoll*-3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2+cosRoll*4.640810924824413E-2+sinRoll*1.856324369929765E-2+((sinPitchMinusRoll*3.81E-2+sinPitchPlusRoll*3.81E-2-cosRoll*2.032E-2+sinRoll*5.08E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2)*2.0-(cosPitchMinusRoll*-1.549666610118799E-2+cosPitchPlusRoll*1.549666610118799E-2+cosRoll*2.066222146825065E-2+sinRoll*8.26488858730026E-3)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0)*1.0/Math.sqrt(-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2,2.0)-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
//           J2A_22=cosPitchMinusRoll*3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2+sinPitch*3.099333220237597E-2-((sinPitchMinusRoll*3.81E-2-sinPitchPlusRoll*3.81E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2)*2.0+(cosPitchMinusRoll*1.549666610118799E-2+cosPitchPlusRoll*1.549666610118799E-2-sinPitch*6.961216387236619E-2)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0)*1.0/Math.sqrt(-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2,2.0)-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
           
           J2A_11 = cosPitchMinusRoll*3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2+sinPitch*3.099333220237597E-2-((sinPitchMinusRoll*3.81E-2-sinPitchPlusRoll*3.81E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2)*2.0+(cosPitchMinusRoll*1.549666610118799E-2+cosPitchPlusRoll*1.549666610118799E-2-sinPitch*6.961216387236619E-2)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0)*1.0/Math.sqrt(-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2,2.0)-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
           J2A_12 = cosPitchMinusRoll*-3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2+cosRoll*4.640810924824413E-2+sinRoll*1.856324369929765E-2+((sinPitchMinusRoll*3.81E-2+sinPitchPlusRoll*3.81E-2-cosRoll*2.032E-2+sinRoll*5.08E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2)*2.0-(cosPitchMinusRoll*-1.549666610118799E-2+cosPitchPlusRoll*1.549666610118799E-2+cosRoll*2.066222146825065E-2+sinRoll*8.26488858730026E-3)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0)*1.0/Math.sqrt(-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2+cosRoll*5.08E-2+sinRoll*2.032E-2-6.985123E-2,2.0)-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3+sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
           J2A_21 = cosPitchMinusRoll*3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2+sinPitch*3.099333220237597E-2-((cosPitchMinusRoll*1.549666610118799E-2+cosPitchPlusRoll*1.549666610118799E-2-sinPitch*6.961216387236619E-2)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0+(sinPitchMinusRoll*3.81E-2-sinPitchPlusRoll*3.81E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2)*2.0)*1.0/Math.sqrt(-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);
           J2A_22 = cosPitchMinusRoll*-3.48060819361831E-2+cosPitchPlusRoll*3.48060819361831E-2-cosRoll*4.640810924824413E-2+sinRoll*1.856324369929765E-2+((sinPitchMinusRoll*3.81E-2+sinPitchPlusRoll*3.81E-2-cosRoll*2.032E-2-sinRoll*5.08E-2)*(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2)*2.0+(cosPitchMinusRoll*1.549666610118799E-2-cosPitchPlusRoll*1.549666610118799E-2+cosRoll*2.066222146825065E-2-sinRoll*8.26488858730026E-3)*(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2)*2.0)*1.0/Math.sqrt(-Math.pow(sinPitchMinusRoll*1.549666610118799E-2+sinPitchPlusRoll*1.549666610118799E-2+cosPitch*6.961216387236619E-2-cosRoll*8.26488858730026E-3-sinRoll*2.066222146825065E-2-4.151172427825791E-2,2.0)-Math.pow(cosPitchMinusRoll*-3.81E-2+cosPitchPlusRoll*3.81E-2-cosRoll*5.08E-2+sinRoll*2.032E-2+6.985123E-2,2.0)+2.473179621695429E15/1.441151880758559E17)*(1.0/2.0);

           
           //this bizarre ordering means that actuators 0 and 1 were switched in the waist and that compression of the force sensors makes Val lean forward in the negative theta direction
           output[0][0]=J2A_11; output[0][1]=-J2A_12;
           output[1][0]=J2A_21; output[1][1]=-J2A_22;
           break;  
           
        }
      }
      case WRIST:
         System.out.println("WRIST jacobians not done for EfficientPushRodTransmission yet.");
         break;
      
      default:
         System.out.println("Invalid pushrodTransmissionJoint.");
         break;
      }
      if(DEBUG)System.out.println(output[0][0] +", " + output[0][1] +", " + output[1][0] +", " + output[1][1]);
      return output;
   }
   
   public void useFuteks(boolean bool){
      this.useFuteks=bool;
//      System.out.println(useFuteks);
   }
   
   public boolean isUsingFuteks(){
	   return useFuteks;
   }
   
   public double cosineOfTheta5(){
	   double cosine;
	   cosine = Math.abs(t5SecondFrame.get(2)-b5SecondFrame.get(2))/length;
	   return cosine;
   }
   
   public double cosineOfTheta6(){
	   double cosine;
	   cosine = Math.abs(t6SecondFrame.get(2)-b6SecondFrame.get(2))/length;
	   return cosine;
   }

}
