package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.*;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

/**
 * Created by agrabertilton on 4/13/15.
 */
public class LegJointLimitAvoidanceControlModule
{
   private static final int maxIterationsForIK = 5;
   private static final double lambdaLeastSquares = 0.0009;
   private static final double tolerance = 1e-12;
   private static final double maxStepSize = 0.01;
   private static final double minRandomSearchScalar = 1.0;
   private static final double maxRandomSearchScalar = 1.0;

   private final DoubleYoVariable percentJointRangeForThreshold;
   private FullRobotModel robotModel;
   private RigidBody base;
   private OneDoFJoint[] robotJoints;
   private OneDoFJoint[] ikJoints;
   private NumericalInverseKinematicsCalculator inverseKinematicsCalculator;
   private GeometricJacobian jacobian;
   private int numJoints;

   private DoubleYoVariable[] originalDesiredPositions;
   private DoubleYoVariable[] alphas;
   private DoubleYoVariable[] adjustedDesiredPositions;
   private YoFramePose originalDesiredYoPose;
   private FramePose originalDesiredPose;
   private YoFramePose adjustedDesiredPose;
   private RigidBodyTransform desiredTransform;

   private final LinearSolver<DenseMatrix64F> solver;
   private final DenseMatrix64F jacobianMatrix;
   private final DenseMatrix64F jacobianMatrixTransposed;
   private final DenseMatrix64F jacobianTimesJaconianTransposedMatrix;
   private final DenseMatrix64F lamdaSquaredMatrix;
   private final DenseMatrix64F jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix;
   private final DenseMatrix64F originalDesiredVelocity;
   private final DenseMatrix64F intermediateResult;
   private final DenseMatrix64F jointVelocities;
   private final DenseMatrix64F adjustedDesiredVelocity;

   public LegJointLimitAvoidanceControlModule(String prefix, YoVariableRegistry registry, MomentumBasedController momentumBasedController, RobotSide robotSide){
      percentJointRangeForThreshold = new DoubleYoVariable(prefix + "percentJointRangeForThreshold", registry);
      robotModel = momentumBasedController.getFullRobotModel();
      base = robotModel.getPelvis();
      RigidBody foot = robotModel.getFoot(robotSide);
      robotJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(base, foot), OneDoFJoint.class);
      ikJoints = ScrewTools.filterJoints(ScrewTools.cloneJointPath(robotJoints), OneDoFJoint.class);
      jacobian = new GeometricJacobian(ikJoints, ikJoints[ikJoints.length-1].getSuccessor().getBodyFixedFrame());
      inverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterationsForIK, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);

      numJoints = ikJoints.length;
      {
         originalDesiredPositions = new DoubleYoVariable[numJoints];
         alphas = new DoubleYoVariable[numJoints];
         adjustedDesiredPositions = new DoubleYoVariable[numJoints];
         for (int i = 0; i < numJoints; i++){
            originalDesiredPositions[i] = new DoubleYoVariable(prefix+ "originalDesiredPositions" +i, registry);
            alphas[i] = new DoubleYoVariable(prefix + "alpha"+ i, registry);
            adjustedDesiredPositions[i] = new DoubleYoVariable(prefix + "adjustedDesiredPositions" +i, registry);
         }

         originalDesiredPose = new FramePose();
         originalDesiredYoPose = new YoFramePose(prefix+"originalDesiredYoPose", ReferenceFrame.getWorldFrame(), registry);
         adjustedDesiredPose = new YoFramePose(prefix+"adjustedDesiredPose", ReferenceFrame.getWorldFrame(), registry);
         desiredTransform = new RigidBodyTransform();
      }

      percentJointRangeForThreshold.set(0.05);


      jacobianMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, numJoints);
      jacobianMatrixTransposed = new DenseMatrix64F(numJoints, SpatialMotionVector.SIZE);
      jacobianTimesJaconianTransposedMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

      lamdaSquaredMatrix = new DenseMatrix64F(numJoints, numJoints);
      jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix = new DenseMatrix64F(numJoints, numJoints);

      solver = LinearSolverFactory.leastSquares(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      originalDesiredVelocity = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
      intermediateResult = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

      jointVelocities = new DenseMatrix64F(numJoints, 1);
      adjustedDesiredVelocity = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   }

   public void correctSwingFootTrajectory(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocityOfOrigin,
                                          FrameVector desiredAngularVelocity, FrameVector desiredLinearAccelerationOfOrigin, FrameVector desiredAngularAcceleration)

   {
      originalDesiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      originalDesiredPose.setPose(desiredPosition, desiredOrientation);
      originalDesiredYoPose.set(originalDesiredPose);
      originalDesiredPose.changeFrame(base.getBodyFixedFrame());
      originalDesiredPose.getPose(desiredTransform);

      inverseKinematicsCalculator.solve(desiredTransform); //sets the qs of the one-dof ikJoints

      // adjust joints based on joint angle limits
//      adjustJointPositions();

      //calcualte the new desired position and orientation
      ikJoints[0].updateFramesRecursively();
      ReferenceFrame adjustedFootFrame = jacobian.getEndEffectorFrame();

      desiredPosition.setToZero(adjustedFootFrame);
      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      desiredOrientation.setToZero(adjustedFootFrame);
      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      adjustedDesiredPose.setPosition(desiredPosition);
      adjustedDesiredPose.setOrientation(desiredOrientation);

      //calculate the adjusted joint velocities using the alphas, then calculate the adjusted velocities
      MatrixTools.setDenseMatrixFromTuple3d(originalDesiredVelocity, desiredAngularVelocity.getVector(), 0, 0);
      MatrixTools.setDenseMatrixFromTuple3d(originalDesiredVelocity, desiredLinearVelocityOfOrigin.getVector(), 3, 0);
      calculateAdjustedVelocities();
      double[] adjustedVelocities = adjustedDesiredVelocity.getData();
      desiredAngularVelocity.set(adjustedVelocities[0], adjustedVelocities[1], adjustedVelocities[2]);
      desiredLinearVelocityOfOrigin.set(adjustedVelocities[3], adjustedVelocities[4], adjustedVelocities[5]);
   }

   private void adjustJointPositions()
   {
      int size = ikJoints.length;
      double upperLimit;
      double lowerLimit;
      double midpointOfLimits;
      double range;
      double rangePercentageForThreshold = percentJointRangeForThreshold.getDoubleValue();
      double lambda = 1- rangePercentageForThreshold;

      for (int i = 0; i < size; i++){
         lowerLimit = ikJoints[i].getJointLimitLower();
         upperLimit = ikJoints[i].getJointLimitUpper();
         midpointOfLimits = (lowerLimit + upperLimit) / 2;
         range = upperLimit - lowerLimit;

         originalDesiredPositions[i].set(ikJoints[i].getQ());
         double comparisonValue = Math.abs(2* (originalDesiredPositions[i].getDoubleValue() - midpointOfLimits) / range); //should range between -1 and 1, which are the limits

         double alpha = 0;
         if (comparisonValue > lambda){
            alpha = (comparisonValue - lambda)/rangePercentageForThreshold;
         }
         alphas[i].set(alpha);

         double adjustedPosition = alpha * robotJoints[i].getQ() + (1-alpha) * ikJoints[i].getqDesired();
         adjustedDesiredPositions[i].set(adjustedPosition);
         ikJoints[i].setqDesired(adjustedPosition);
      }
   }

   private void calculateAdjustedVelocities(){

      int numberOfConstraints = SpatialMotionVector.SIZE;

      // J
      jacobianMatrix.set(jacobian.getJacobianMatrix());

      // J^T
      CommonOps.transpose(jacobianMatrix, jacobianMatrixTransposed);

      // J J^T
      CommonOps.multOuter(jacobianMatrix, jacobianTimesJaconianTransposedMatrix);


      intermediateResult.reshape(numberOfConstraints, 1);

      lamdaSquaredMatrix.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps.setIdentity(lamdaSquaredMatrix);
      CommonOps.scale(lambdaLeastSquares, lamdaSquaredMatrix);

      jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix.reshape(numberOfConstraints, numberOfConstraints);
      jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix.set(jacobianTimesJaconianTransposedMatrix);
      CommonOps.add(jacobianTimesJaconianTransposedMatrix, lamdaSquaredMatrix, jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix);

      boolean success = solver.setA(jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix);

      // Solve J*J^T deltaX = f
      solver.solve(originalDesiredVelocity, intermediateResult);
      CommonOps.mult(jacobianMatrixTransposed, intermediateResult, jointVelocities);

      for (int i = 0; i < numJoints; i++){
         jointVelocities.times(i, alphas[i].getDoubleValue());
      }

      CommonOps.mult(jacobianMatrix, jointVelocities, adjustedDesiredVelocity);
   }
}