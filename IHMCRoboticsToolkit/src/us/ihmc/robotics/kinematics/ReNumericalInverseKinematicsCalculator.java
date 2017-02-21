package us.ihmc.robotics.kinematics;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;

/**
 * @author twan
 *         Date: 6/1/13
 */
public class ReNumericalInverseKinematicsCalculator implements InverseKinematicsCalculator
{
   private final GeometricJacobian jacobian;
   private final LinearSolver<DenseMatrix64F> solver;
   private final OneDoFJoint[] oneDoFJoints;
   private final OneDoFJoint[] oneDoFJointsSeed;

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

   private final DenseMatrix64F error = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F correction;
   private final DenseMatrix64F current;
   private final DenseMatrix64F best;
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
      this.solver = LinearSolverFactory.leastSquares(SpatialMotionVector.SIZE, jacobian.getNumberOfColumns()); // new DampedLeastSquaresSolver(jacobian.getNumberOfColumns());

      this.oneDoFJoints = ScrewTools.filterJoints(jacobian.getJointsInOrder(), OneDoFJoint.class);
      oneDoFJointsSeed = oneDoFJoints.clone();

      if (oneDoFJoints.length != jacobian.getJointsInOrder().length)
         throw new RuntimeException("Can currently only handle OneDoFJoints");
      int nDoF = ScrewTools.computeDegreesOfFreedom(oneDoFJoints);
      correction = new DenseMatrix64F(nDoF, 1);
      current = new DenseMatrix64F(nDoF, 1);
      best = new DenseMatrix64F(nDoF, 1);
      this.tolerance = tolerance;
      this.maxIterations = maxIterations;
      this.maxStepSize = maxStepSize;
      this.minRandomSearchScalar = minRandomSearchScalar;
      this.maxRandomSearchScalar = maxRandomSearchScalar;
      counter = 0;
   }

   public boolean solve(RigidBodyTransform desiredTransform)
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

   public void introduceRandomArmePose(RigidBodyTransform desiredTransform)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJoints[i].setQ(random.nextDouble());
      }
      solve(desiredTransform);
   }

   private void getJointAngles(DenseMatrix64F angles)
   {
      for (int i = 0; i < angles.getNumRows(); i++)
      {
         angles.set(i, 0, oneDoFJoints[i].getQ());
      }
   }

   private void setJointAngles(DenseMatrix64F angles)
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

   private void computeError(RigidBodyTransform desiredTransform)
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

      errorTransform.getRotation(errorRotationMatrix);
      errorAxisAngle.set(errorRotationMatrix);

      axis.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.set(axis);
      errorRotationVector.scale(errorAxisAngle.getAngle());

      errorTransform.getTranslation(errorTranslationVector);
      
      errorRotationVector.get(0, error);
      errorTranslationVector.get(3, error);

      errorScalar = NormOps.normP2(error);

      assert (exponentialCoordinatesOK());
   }

   private boolean exponentialCoordinatesOK()
   {
      Twist twist = new Twist(jacobian.getEndEffectorFrame(), jacobian.getBaseFrame(), jacobian.getJacobianFrame(), error);
      RotationMatrix rotationCheck = new RotationMatrix();
      rotationCheck.setIdentity();
      Vector3D positionCheck = new Vector3D();
      ScrewTestTools.integrate(rotationCheck, positionCheck, 1.0, twist);
      RigidBodyTransform transformCheck = new RigidBodyTransform(rotationCheck, positionCheck);

      return transformCheck.epsilonEquals(errorTransform, 1e-5);
   }

   private void computeCorrection()
   {
      jacobian.compute();

      DenseMatrix64F dampenedJ = jacobian.getJacobianMatrix().copy();

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
      double correctionScale = RandomTools.generateRandomDouble(random, minRandomSearchScalar, maxRandomSearchScalar);
      CommonOps.scale(correctionScale, correction);

      for (int i = 0; i < correction.getNumRows(); i++)
      {
         correction.set(i, 0, MathTools.clipToMinMax(correction.get(i, 0), -maxStepSize, maxStepSize));
      }
   }

   private void applyCorrection()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint;
         if (useSeed)
         {
            oneDoFJoint = oneDoFJointsSeed[i];
         }
         else
         {
            oneDoFJoint = oneDoFJoints[i];
         }
         double newQ = oneDoFJoint.getQ() - correction.get(i, 0);
         newQ = MathTools.clipToMinMax(newQ, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
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
      // TODO Auto-generated method stub
      
   }
}
