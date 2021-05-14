package us.ihmc.robotics.kinematics;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class KinematicSolver implements InverseKinematicsCalculator
{
   private static long seed = 129092439L;
   private static final Random randomNumberGenerator = new Random(seed);
   private static int nDoF = 0;
   private final ArrayList<Double> errorCourse = new ArrayList<Double>();

   private final LinearSolverDense<DMatrixRMaj> getPseudoInverse = LinearSolverFactory_DDRM.pseudoInverse(true);
   private final RigidBodyTransform transformShoulderToEndEffector = new RigidBodyTransform();
   private final RigidBodyTransform transformEndEffectorToShoulder = new RigidBodyTransform();
   private final RigidBodyTransform transformEndEffectorToDesired = new RigidBodyTransform();
   private final Vector3D errorTranslationVector = new Vector3D();
   private final Vector3D errorAngularVector = new Vector3D();
   private final AxisAngle errorAxisAngle = new AxisAngle();
   private final Quaternion errorQuat = new Quaternion();

   private final DMatrixRMaj spatialError;
   private final DMatrixRMaj jacobianTranspose;
   private final DMatrixRMaj jacobianJacobianTranspose;
   private final DMatrixRMaj correction;
   private final DMatrixRMaj jacobianMethod;
   private final DMatrixRMaj jJTe;
   private final DMatrixRMaj dampingSquaredDiagonal;
   private final DMatrixRMaj inverseTerm;

   private final GeometricJacobian jacobian;
   private final OneDoFJointBasics[] oneDoFJoints;

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
      this.oneDoFJoints = MultiBodySystemTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJointBasics.class);
      nDoF = MultiBodySystemTools.computeDegreesOfFreedom(oneDoFJoints);

      jacobianMethod = new DMatrixRMaj(nDoF, nDoF);
      jacobianTranspose = new DMatrixRMaj(nDoF, nDoF);
      jacobianJacobianTranspose = new DMatrixRMaj(nDoF, nDoF);
      jJTe = new DMatrixRMaj(nDoF, 1);
      correction = new DMatrixRMaj(nDoF, 1);
      spatialError = new DMatrixRMaj(SpatialVector.SIZE, 1);
      dampingSquaredDiagonal = new DMatrixRMaj(nDoF, nDoF);
      inverseTerm = new DMatrixRMaj(nDoF, nDoF);

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
      CommonOps_DDRM.mult(jacobianMethod, spatialError, correction);
   }

   private void monitorProgress()
   {
      double converge = 10;
      double spatialErrorNorm = NormOps_DDRM.normP2(spatialError);
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
         newQ = MathTools.clamp(newQ, oneDoFJoints[i].getJointLimitLower(), oneDoFJoints[i].getJointLimitUpper());
         oneDoFJoints[i].setQ(newQ);
         oneDoFJoints[i].getFrameAfterJoint().update();
      }
   }

   private boolean correctionNearZero()
   {
      if (NormOps_DDRM.normP2(spatialError) < tolerance)
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

   private DMatrixRMaj getUpdatedJacobianMatrix()
   {
      jacobian.compute();

      return jacobian.getJacobianMatrix();
   }

   private void jacobianOperation(JacobianMethod type)
   {
      CommonOps_DDRM.transpose(jacobianMethod, jacobianTranspose);
      CommonOps_DDRM.mult(jacobianMethod, jacobianTranspose, jacobianJacobianTranspose);

      switch (type)
      {
         case JACOBIAN_INVERSE :

            // J^-1
            CommonOps_DDRM.invert(jacobianMethod);

            break;

         case JACOBIAN_TRANSPOSE :

            // alpha = <spatialError, J*J^T * spatialError> / <J*J^T * spatialError, J*J^T * spatialError>
            CommonOps_DDRM.mult(jacobianJacobianTranspose, spatialError, jJTe);
            alpha = getDotProduct(spatialError, jJTe) / getDotProduct(jJTe, jJTe);
            CommonOps_DDRM.transpose(jacobianMethod);
            CommonOps_DDRM.scale(alpha, jacobianMethod);

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
            CommonOps_DDRM.setIdentity(dampingSquaredDiagonal);
            CommonOps_DDRM.scale(dampingConstant * dampingConstant, dampingSquaredDiagonal);

            // (J*J^T + lambda^2*I)
            CommonOps_DDRM.add(jacobianJacobianTranspose, dampingSquaredDiagonal, inverseTerm);

            // (J*J^T + lambda^2*I)^-1
            CommonOps_DDRM.invert(inverseTerm);

            // J^T * (J*J^T + lambda^2*I)^-1
            CommonOps_DDRM.mult(jacobianTranspose, inverseTerm, jacobianMethod);

            break;
      }
   }

   private double getDotProduct(DMatrixRMaj a, DMatrixRMaj b)
   {
      double ret = 0;
      for (int i = 0; i < a.numRows; i++)
      {
         ret += (a.get(i, 0) * b.get(i, 0));
      }

      return ret;
   }

   private DMatrixRMaj getSpatialErrorEndEffectorDesired()
   {
      errorTranslationVector.set(transformEndEffectorToDesired.getTranslation());
      errorQuat.set(transformEndEffectorToDesired.getRotation());
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
      return NormOps_DDRM.normF(spatialError);
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
      
   }
}
