package us.ihmc.robotics.kinematics;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

/**
 * @author twan
 *         Date: 6/1/13
 */
public class ReNumericalInverseKinematicsCalculator implements InverseKinematicsCalculator
{
   private final GeometricJacobian jacobian;
   private final LinearSolverDense<DMatrixRMaj> solver;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final OneDoFJointBasics[] oneDoFJointsSeed;

   private final double tolerance;
   private final int maxIterations;
   private final double maxStepSize;
   private final Random random = new Random(1251253L);

   private int iterationNumber;
   private double errorScalar;
   private double bestErrorScalar;

   private final RigidBodyTransform actualTransform = new RigidBodyTransform();
   private final RigidBodyTransform errorTransform = new RigidBodyTransform();
   private final AxisAngle errorAxisAngle = new AxisAngle();
   private final RotationMatrix errorRotationMatrix = new RotationMatrix();
   private final Vector3D errorRotationVector = new Vector3D();
   private final Vector3D axis = new Vector3D();
   private final Vector3D errorTranslationVector = new Vector3D();

   private final DMatrixRMaj error = new DMatrixRMaj(SpatialVector.SIZE, 1);
   private final DMatrixRMaj correction;
   private final DMatrixRMaj current;
   private final DMatrixRMaj best;
   private final double minRandomSearchScalar;
   private final double maxRandomSearchScalar;
   private int counter;
   private boolean useSeed;

   public ReNumericalInverseKinematicsCalculator(GeometricJacobian jacobian, double tolerance, int maxIterations, double maxStepSize,
         double minRandomSearchScalar, double maxRandomSearchScalar)
   {
      if (jacobian.getJacobianFrame() != jacobian.getEndEffectorFrame())
         throw new RuntimeException("jacobian.getJacobianFrame() != jacobian.getEndEffectorFrame()");
      this.jacobian = jacobian;
      this.solver = LinearSolverFactory_DDRM.leastSquares(SpatialVector.SIZE, jacobian.getNumberOfColumns()); // new DampedLeastSquaresSolver(jacobian.getNumberOfColumns());

      this.oneDoFJoints = MultiBodySystemTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJointBasics.class);
      oneDoFJointsSeed = oneDoFJoints.clone();

      if (oneDoFJoints.length != jacobian.getJointsInOrder().length)
         throw new RuntimeException("Can currently only handle OneDoFJoints");
      int nDoF = MultiBodySystemTools.computeDegreesOfFreedom(oneDoFJoints);
      correction = new DMatrixRMaj(nDoF, 1);
      current = new DMatrixRMaj(nDoF, 1);
      best = new DMatrixRMaj(nDoF, 1);
      this.tolerance = tolerance;
      this.maxIterations = maxIterations;
      this.maxStepSize = maxStepSize;
      this.minRandomSearchScalar = minRandomSearchScalar;
      this.maxRandomSearchScalar = maxRandomSearchScalar;
      counter = 0;
   }

   public boolean solve(RigidBodyTransformReadOnly desiredTransform)
   {
      iterationNumber = 0;

      bestErrorScalar = Double.POSITIVE_INFINITY;

      do
      {
         computeError(desiredTransform);
         updateBest();
         computeCorrection();
         applyCorrection();

         iterationNumber++;
      }
      while ((errorScalar > tolerance) && (iterationNumber < maxIterations));

      updateBest();
      setJointAngles(best);
      if (iterationNumber < maxIterations)
      {
         useSeed = false;
         return true;
      }

      if (iterationNumber >= maxIterations && counter++ < 100)
      {
         introduceRandomArmePose(desiredTransform);
      }
      counter = 0;
      return false;
   }

   public void introduceRandomArmePose(RigidBodyTransformReadOnly desiredTransform)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJoints[i].setQ(random.nextDouble());
      }
      solve(desiredTransform);
   }

   private void getJointAngles(DMatrixRMaj angles)
   {
      for (int i = 0; i < angles.getNumRows(); i++)
      {
         angles.set(i, 0, oneDoFJoints[i].getQ());
      }
   }

   private void setJointAngles(DMatrixRMaj angles)
   {
      for (int i = 0; i < angles.getNumRows(); i++)
      {
         oneDoFJoints[i].setQ(angles.get(i, 0));
      }
   }

   private void updateBest()
   {
      getJointAngles(current);

      if (errorScalar < bestErrorScalar)
      {
         best.set(current);
         bestErrorScalar = errorScalar;
      }
   }

   private void computeError(RigidBodyTransformReadOnly desiredTransform)
   {
      /*
       * B is base E is end effector D is desired end effector
       * 
       * H^D_E = H^D_B * H^B_E = (H^B_D)^-1 * H^B_E
       * 
       * convert rotation matrix part to rotation vector
       */

      jacobian.getEndEffectorFrame().getTransformToDesiredFrame(actualTransform, jacobian.getBaseFrame());

      errorTransform.setAndInvert(desiredTransform);
      errorTransform.multiply(actualTransform);

      errorRotationMatrix.set(errorTransform.getRotation());
      errorAxisAngle.set(errorRotationMatrix);

      axis.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.set(axis);
      errorRotationVector.scale(errorAxisAngle.getAngle());

      errorTranslationVector.set(errorTransform.getTranslation());
      
      errorRotationVector.get(0, error);
      errorTranslationVector.get(3, error);

      errorScalar = NormOps_DDRM.normP2(error);

      assert (exponentialCoordinatesOK());
   }

   private boolean exponentialCoordinatesOK()
   {
      Twist twist = new Twist(jacobian.getEndEffectorFrame(), jacobian.getBaseFrame(), jacobian.getJacobianFrame(), error);
      Pose3D poseCheck = new Pose3D();
      new MultiBodySystemStateIntegrator(1.0).integrate(twist, poseCheck);
      RigidBodyTransform transformCheck = new RigidBodyTransform(poseCheck.getOrientation(), poseCheck.getPosition());

      return transformCheck.epsilonEquals(errorTransform, 1e-5);
   }

   private void computeCorrection()
   {
      jacobian.compute();

      DMatrixRMaj dampenedJ = jacobian.getJacobianMatrix().copy();

      for (int i = 0; i < dampenedJ.getNumCols(); i++)
      {
         dampenedJ.add(i, i, 0.0);
      }

      //    solver.setAlpha(errorScalar + dampedLeastSquaresAlphaMin);
      if (!solver.setA(dampenedJ))
      {
         // do something here intelligent if it fails
         //         System.err.println("IK solver internal solve failed!");
      }
      solver.solve(error, correction);
      double correctionScale = RandomNumbers.nextDouble(random, minRandomSearchScalar, maxRandomSearchScalar);
      CommonOps_DDRM.scale(correctionScale, correction);

      for (int i = 0; i < correction.getNumRows(); i++)
      {
         correction.set(i, 0, MathTools.clamp(correction.get(i, 0), -maxStepSize, maxStepSize));
      }
   }

   private void applyCorrection()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics oneDoFJoint;
         if (useSeed)
         {
            oneDoFJoint = oneDoFJointsSeed[i];
         }
         else
         {
            oneDoFJoint = oneDoFJoints[i];
         }
         double newQ = oneDoFJoint.getQ() - correction.get(i, 0);
         newQ = MathTools.clamp(newQ, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         oneDoFJoint.setQ(newQ);
         oneDoFJoint.getFrameAfterJoint().update();
      }
   }

   public int getNumberOfIterations()
   {
      return iterationNumber;
   }

   public double getErrorScalar()
   {
      return errorScalar;
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
