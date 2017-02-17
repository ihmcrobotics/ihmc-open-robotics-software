package us.ihmc.robotics.kinematics;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

public class KinematicSolver implements InverseKinematicsCalculator
{
   private static long seed = 129092439L;
   private static final Random randomNumberGenerator = new Random(seed);
   private static int nDoF = 0;
   private final ArrayList<Double> errorCourse = new ArrayList<Double>();

   private final LinearSolver<DenseMatrix64F> getPseudoInverse = LinearSolverFactory.pseudoInverse(true);
   private final RigidBodyTransform transformShoulderToEndEffector = new RigidBodyTransform();
   private final RigidBodyTransform transformEndEffectorToShoulder = new RigidBodyTransform();
   private final RigidBodyTransform transformEndEffectorToDesired = new RigidBodyTransform();
   private final Vector3D errorTranslationVector = new Vector3D();
   private final Vector3D errorAngularVector = new Vector3D();
   private final AxisAngle errorAxisAngle = new AxisAngle();
   private final Quaternion errorQuat = new Quaternion();

   private final DenseMatrix64F spatialError;
   private final DenseMatrix64F jacobianTranspose;
   private final DenseMatrix64F jacobianJacobianTranspose;
   private final DenseMatrix64F correction;
   private final DenseMatrix64F jacobianMethod;
   private final DenseMatrix64F jJTe;
   private final DenseMatrix64F dampingSquaredDiagonal;
   private final DenseMatrix64F inverseTerm;

   private final GeometricJacobian jacobian;
   private final OneDoFJoint[] oneDoFJoints;

   private final double dampingConstant;

   private final double tolerance;
   private final double maxIterations;
   private final double convergeRate;
   private final double[] bestJointAngles = new double[6];

   private int iterationNumber;
   private double currentBest;
   private double alpha;
   private boolean desiredReached;
   private JacobianMethod solveMethod;

   public enum JacobianMethod {JACOBIAN_INVERSE, JACOBIAN_TRANSPOSE, PSEUDO_INVERSE, DAMPED_LEAST_SQUARES}

   ;

   public KinematicSolver(GeometricJacobian jacobian, double tolerance, double maxIterations)
   {
      this.jacobian = jacobian;
      this.tolerance = tolerance;
      this.maxIterations = maxIterations;
      this.oneDoFJoints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      nDoF = ScrewTools.computeDegreesOfFreedom(oneDoFJoints);

      jacobianMethod = new DenseMatrix64F(nDoF, nDoF);
      jacobianTranspose = new DenseMatrix64F(nDoF, nDoF);
      jacobianJacobianTranspose = new DenseMatrix64F(nDoF, nDoF);
      jJTe = new DenseMatrix64F(nDoF, 1);
      correction = new DenseMatrix64F(nDoF, 1);
      spatialError = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
      dampingSquaredDiagonal = new DenseMatrix64F(nDoF, nDoF);
      inverseTerm = new DenseMatrix64F(nDoF, nDoF);

      convergeRate = 1E-8;
      solveMethod = JacobianMethod.JACOBIAN_TRANSPOSE;
      dampingConstant = 0.3;
   }

   public boolean solve(RigidBodyTransform desiredTransform)
   {
      currentBest = Double.MAX_VALUE;
      iterationNumber = 0;
      errorCourse.clear();
      desiredReached = false;
      do
      {
         calculateErrorTransform(desiredTransform);
         minimizeError();
         updateJointAngles();
         monitorProgress();
         desiredReached = correctionNearZero();
      }
      while ((iterationNumber++ < maxIterations) &&!desiredReached);

      setBestJointAngles();

      return desiredReached;
   }

   private void calculateErrorTransform(RigidBodyTransform transformShoulderToDesired)
   {
      jacobian.getEndEffector().getBodyFixedFrame().getTransformToDesiredFrame(transformShoulderToEndEffector, jacobian.getBaseFrame());
      transformEndEffectorToShoulder.setAndInvert(transformShoulderToEndEffector);
      transformEndEffectorToDesired.set(transformEndEffectorToShoulder);
      transformEndEffectorToDesired.multiply(transformShoulderToDesired);
   }

   private void minimizeError()
   {
      jacobianMethod.set(getUpdatedJacobianMatrix());
      spatialError.set(getSpatialErrorEndEffectorDesired());
      jacobianOperation(solveMethod);
      CommonOps.mult(jacobianMethod, spatialError, correction);
   }

   private void monitorProgress()
   {
      double converge = 10;
      double spatialErrorNorm = NormOps.normP2(spatialError);
      errorCourse.add(spatialErrorNorm);
      int lastIndex = errorCourse.size();

      int limit = 10;
      if (lastIndex >= limit)
      {
         converge = errorCourse.get(0) - errorCourse.get(limit - 1);
         errorCourse.remove(0);
      }

      if (converge < convergeRate)
      {
         introduceRandomPose();
      }

      checkAndSetIfPoseIsBest(spatialErrorNorm);
   }

   private void updateJointAngles()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         double newQ = oneDoFJoints[i].getQ() + correction.get(i, 0);
         newQ = MathTools.clipToMinMax(newQ, oneDoFJoints[i].getJointLimitLower(), oneDoFJoints[i].getJointLimitUpper());
         oneDoFJoints[i].setQ(newQ);
         oneDoFJoints[i].getFrameAfterJoint().update();
      }
   }

   private boolean correctionNearZero()
   {
      if (NormOps.normP2(spatialError) < tolerance)
      {
         return true;
      }

      return false;
   }

   private void setBestJointAngles()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJoints[i].setQ(bestJointAngles[i]);
      }
   }

   private DenseMatrix64F getUpdatedJacobianMatrix()
   {
      jacobian.compute();

      return jacobian.getJacobianMatrix();
   }

   private void jacobianOperation(JacobianMethod type)
   {
      CommonOps.transpose(jacobianMethod, jacobianTranspose);
      CommonOps.mult(jacobianMethod, jacobianTranspose, jacobianJacobianTranspose);

      switch (type)
      {
         case JACOBIAN_INVERSE :

            // J^-1
            CommonOps.invert(jacobianMethod);

            break;

         case JACOBIAN_TRANSPOSE :

            // alpha = <spatialError, J*J^T * spatialError> / <J*J^T * spatialError, J*J^T * spatialError>
            CommonOps.mult(jacobianJacobianTranspose, spatialError, jJTe);
            alpha = getDotProduct(spatialError, jJTe) / getDotProduct(jJTe, jJTe);
            CommonOps.transpose(jacobianMethod);
            CommonOps.scale(alpha, jacobianMethod);

            break;

         case PSEUDO_INVERSE :
            if (!getPseudoInverse.setA(jacobianMethod))
            {
               throw new RuntimeException("PseudoInverse failed");
            }

            getPseudoInverse.invert(jacobianMethod);

            break;

         case DAMPED_LEAST_SQUARES :

            // lambda^2*I
            CommonOps.setIdentity(dampingSquaredDiagonal);
            CommonOps.scale(dampingConstant * dampingConstant, dampingSquaredDiagonal);

            // (J*J^T + lambda^2*I)
            CommonOps.add(jacobianJacobianTranspose, dampingSquaredDiagonal, inverseTerm);

            // (J*J^T + lambda^2*I)^-1
            CommonOps.invert(inverseTerm);

            // J^T * (J*J^T + lambda^2*I)^-1
            CommonOps.mult(jacobianTranspose, inverseTerm, jacobianMethod);

            break;
      }
   }

   private double getDotProduct(DenseMatrix64F a, DenseMatrix64F b)
   {
      double ret = 0;
      for (int i = 0; i < a.numRows; i++)
      {
         ret += (a.get(i, 0) * b.get(i, 0));
      }

      return ret;
   }

   private DenseMatrix64F getSpatialErrorEndEffectorDesired()
   {
      transformEndEffectorToDesired.getTranslation(errorTranslationVector);
      transformEndEffectorToDesired.getRotation(errorQuat);
      errorAxisAngle.set(errorQuat);
      errorAngularVector.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorAngularVector.scale(errorAxisAngle.getAngle());
      errorAngularVector.get(0, 0, spatialError);
      errorTranslationVector.get(3, 0, spatialError);

      return spatialError;
   }

   private void introduceRandomPose()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         double newQ = generateRandomDoubleInRange(oneDoFJoints[i].getJointLimitUpper(), oneDoFJoints[i].getJointLimitLower());
         oneDoFJoints[i].setQ(newQ);
      }
   }

   private void checkAndSetIfPoseIsBest(double spatialErrorNorm)
   {
      if (spatialErrorNorm < currentBest)
      {
         currentBest = spatialErrorNorm;

         for (int i = 0; i < bestJointAngles.length; i++)
         {
            bestJointAngles[i] = oneDoFJoints[i].getQ();
         }
      }
   }

   private double generateRandomDoubleInRange(double max, double min)
   {
      return min + (max - min) * randomNumberGenerator.nextDouble();
   }

   public double getErrorScalar()
   {
      return NormOps.normF(spatialError);
   }

   public int getNumberOfIterations()
   {
      return iterationNumber;
   }

   public JacobianMethod getSolveMethod()
   {
      return solveMethod;
   }

   public void setSolveMethod(JacobianMethod solveMethod)
   {
      this.solveMethod = solveMethod;
   }

   public double getDampingConstant()
   {
      return dampingConstant;
   }

   @Override
   public void attachInverseKinematicsStepListener(InverseKinematicsStepListener stepListener)
   {      
   }

   @Override
   public void setLimitJointAngles(boolean limitJointAngles)
   {
      // TODO Auto-generated method stub
      
   }
}
