package us.ihmc.robotics.kinematics;

import java.util.LinkedHashMap;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

/**
 * @author twan
 *         Date: 6/1/13
 */
public class NumericalInverseKinematicsCalculator implements InverseKinematicsCalculator
{
   private final InverseJacobianSolver inverseJacobianCalculator;
   
   private final OneDoFJoint[] oneDoFJoints;
   private final double tolerance;
   private final int maxIterations;
   private final double maxStepSize;
   private final Random random = new Random(1251253L);

   private int iterationNumber;
   private double errorScalar;
   private double minimumErrorScalar;

   private final RigidBodyTransform actualTransform = new RigidBodyTransform();
   private final RigidBodyTransform errorTransform = new RigidBodyTransform();
   private final AxisAngle errorAxisAngle = new AxisAngle();
   private final RotationMatrix errorRotationMatrix = new RotationMatrix();
   private final Vector3D errorRotationVector = new Vector3D();
   private final Vector3D axis = new Vector3D();
   private final Vector3D errorTranslationVector = new Vector3D();

   private final DenseMatrix64F spatialError = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F jointAnglesCorrection;
   private final DenseMatrix64F jointAnglesMinimumError;
   
   private final double minRandomSearchScalar;
   private final double maxRandomSearchScalar;

   private int numberOfConstraints;
   private final int numberOfDoF;

   private InverseKinematicsStepListener stepListener = null;
   private boolean limitJointAngles = true;

   private final GeometricJacobian jacobian;
   private final double lambdaLeastSquares;
   
   public static NumericalInverseKinematicsCalculator createIKCalculator(InverseDynamicsJoint[] jointsToControl, int maxIterations)
   {
      InverseDynamicsJoint[] cloneOfControlledJoints = ScrewTools.cloneJointPath(jointsToControl);

      int numberOfDoFs = cloneOfControlledJoints.length;
      RigidBody cloneOfEndEffector = cloneOfControlledJoints[numberOfDoFs - 1].getSuccessor();
      ReferenceFrame cloneOfEndEffectorFrame = cloneOfEndEffector.getBodyFixedFrame();
      GeometricJacobian jacobian = new GeometricJacobian(cloneOfControlledJoints, cloneOfEndEffectorFrame);

      double lambdaLeastSquares = 0.0009;
      double tolerance = 0.001;
      double maxStepSize = 0.2;
      double minRandomSearchScalar = 0.02;
      double maxRandomSearchScalar = 0.8;

      return new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterations, maxStepSize, minRandomSearchScalar,
            maxRandomSearchScalar);
   }
   
   public NumericalInverseKinematicsCalculator(GeometricJacobian jacobian, double lambdaLeastSquares, double tolerance, int maxIterations, double maxStepSize,
         double minRandomSearchScalar, double maxRandomSearchScalar)
   {
      if (jacobian.getJacobianFrame() != jacobian.getEndEffectorFrame())
         throw new RuntimeException("jacobian.getJacobianFrame() != jacobian.getEndEffectorFrame()");
      
      this.jacobian = jacobian;
      numberOfConstraints = SpatialMotionVector.SIZE;
      numberOfDoF = jacobian.getNumberOfColumns();
      inverseJacobianCalculator = InverseJacobianSolver.createInverseJacobianSolver(numberOfConstraints, numberOfDoF, false);

      oneDoFJoints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      if (oneDoFJoints.length != jacobian.getJointsInOrder().length)
         throw new RuntimeException("Can currently only handle OneDoFJoints");

      jointAnglesCorrection = new DenseMatrix64F(numberOfDoF, 1);
      jointAnglesMinimumError = new DenseMatrix64F(numberOfDoF, 1);

      this.lambdaLeastSquares = lambdaLeastSquares;
      this.tolerance = tolerance;
      this.maxIterations = maxIterations;
      this.maxStepSize = maxStepSize;
      this.minRandomSearchScalar = minRandomSearchScalar;
      this.maxRandomSearchScalar = maxRandomSearchScalar;
   }
  
   @Override
   public void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener)
   {
      this.stepListener = stepListener;
   }

   public void setLimitJointAngles(boolean limitJointAngles)
   {
      this.limitJointAngles = limitJointAngles;
   }
   
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumCols() != SpatialMotionVector.SIZE)
         throw new RuntimeException("The selection matrix must have 6 columns, the argument has: " + selectionMatrix.getNumCols());

      inverseJacobianCalculator.setSelectionMatrix(selectionMatrix);
   }

   /**
    * Solves the desired transform for the joint angles using the equation: &delta;q = J<sup>T</sup>(JJ<sup>T</sup>)<sup>-1</sup> &delta;X, with:
    * <li> &delta;q is the correction in joint space </li>
    * <li> J is the Jacobian </li>
    * <li> &delta;X is the spatial (orientation and position) error in taskspace </li>
    */
   @Override
   public boolean solve(RigidBodyTransform desiredTransform)
   {
      iterationNumber = 0;
      minimumErrorScalar = Double.POSITIVE_INFINITY;

      numberOfConstraints = inverseJacobianCalculator.getNumberOfConstraints();

      boolean hasReachedMaxIterations = false;
      boolean hasReachedTolerance = false;
      do
      {
         computeError(desiredTransform);
         computeJointAngleCorrection(spatialError);
         updateBest();
         
         applyJointAngleCorrection();

         iterationNumber++;
         hasReachedMaxIterations = iterationNumber >= maxIterations;
         hasReachedTolerance = errorScalar <= tolerance;
      }
      while (!hasReachedTolerance && !hasReachedMaxIterations);

      updateBest();
      setJointAngles(jointAnglesMinimumError);

      return hasReachedTolerance;
   }
   
   public void getBest(DenseMatrix64F bestToPack)
   {
      bestToPack.set(jointAnglesMinimumError);
   }
   
   public void getBest(LinkedHashMap<OneDoFJoint, Double> bestToPack)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
         bestToPack.put(oneDoFJoints[i], jointAnglesMinimumError.get(i, 0));
   }

   private void getJointAngles(DenseMatrix64F angles)
   {
      for (int i = 0; i < angles.getNumRows(); i++)
      {
         angles.set(i, 0, oneDoFJoints[i].getQ());
      }
   }

   public void setJointAngles(DenseMatrix64F angles)
   {
      for (int i = 0; i < angles.getNumRows(); i++)
      {
         oneDoFJoints[i].setQ(angles.get(i, 0));
      }
   }

   private void updateBest()
   {
      errorScalar = NormOps.normP2(inverseJacobianCalculator.getSubspaceSpatialVelocity());

      if (errorScalar < minimumErrorScalar)
      {
         getJointAngles(jointAnglesMinimumError);
         minimumErrorScalar = errorScalar;
      }
   }

   private void computeError(RigidBodyTransform desiredTransform)
   {
      jacobian.compute();
      jacobian.getEndEffectorFrame().getTransformToDesiredFrame(actualTransform, jacobian.getBaseFrame());

      errorTransform.setAndInvert(desiredTransform);
      errorTransform.multiply(actualTransform);

      errorTransform.getRotation(errorRotationMatrix);
      errorAxisAngle.set(errorRotationMatrix);

      axis.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.set(axis);
      errorRotationVector.scale(errorAxisAngle.getAngle());
      errorRotationVector.scale(0.2);
      
      errorTransform.getTranslation(errorTranslationVector);
      
      errorRotationVector.get(0, 0, spatialError);
      errorTranslationVector.get(3, 0, spatialError);
   }

   private void computeJointAngleCorrection(DenseMatrix64F spatialError)
   {      
//      inverseJacobianCalculator.solveUsingJacobianInverse(spatialError);
//      inverseJacobianCalculator.solveUsingJacobianPseudoInverseOne(spatialError);
//      inverseJacobianCalculator.solveUsingJacobianPseudoInverseTwo(spatialError);
      inverseJacobianCalculator.solveUsingDampedLeastSquares(spatialError, jacobian.getJacobianMatrix(), lambdaLeastSquares);
      
      jointAnglesCorrection.set(inverseJacobianCalculator.getJointspaceVelocity());
      
      double correctionScale = RandomNumbers.nextDouble(random, minRandomSearchScalar, maxRandomSearchScalar);
      CommonOps.scale(correctionScale, jointAnglesCorrection);

      for (int i = 0; i < jointAnglesCorrection.getNumRows(); i++)
      {
         jointAnglesCorrection.set(i, 0, Math.min(maxStepSize, Math.max(jointAnglesCorrection.get(i, 0), -maxStepSize)));
      }
   } 

   private void applyJointAngleCorrection()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         double newQ = oneDoFJoint.getQ() - jointAnglesCorrection.get(i, 0);
         if (limitJointAngles) 
            newQ = Math.min(oneDoFJoint.getJointLimitUpper(), Math.max(newQ, oneDoFJoint.getJointLimitLower()));
         oneDoFJoint.setQ(newQ);
         oneDoFJoint.getFrameAfterJoint().update();
      }
      
      if (stepListener != null)
      {
         stepListener.didAnInverseKinemticsStep(errorScalar);
      }
   }

   @Override
   public int getNumberOfIterations()
   {
      return iterationNumber;
   }

   @Override
   public double getErrorScalar()
   {
      return minimumErrorScalar;
   }
}
