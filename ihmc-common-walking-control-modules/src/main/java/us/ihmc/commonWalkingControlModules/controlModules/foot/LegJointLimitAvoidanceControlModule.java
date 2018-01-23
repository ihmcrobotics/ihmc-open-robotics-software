package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;

/**
 * Created by agrabertilton on 4/13/15.
 */
public class LegJointLimitAvoidanceControlModule
{
   private static final int maxIterationsForIK = 8;
   private static final boolean translationFixOnly = true;
   private boolean visualize = true;
   private boolean enableCorrection = false;

   private static final double lambdaLeastSquares = 0.000001;
   private static final double tolerance = 1.0e-8;
   private static final double maxStepSize = 0.1;
   private static final double minRandomSearchScalar = 1.0;
   private static final double maxRandomSearchScalar = 1.0;

   private final YoDouble percentJointRangeForThreshold;
   private FullHumanoidRobotModel robotModel;
   private RigidBody base;
   private OneDoFJoint[] robotJoints;
   private OneDoFJoint[] ikJoints;
   private NumericalInverseKinematicsCalculator inverseKinematicsCalculator;
   private GeometricJacobian jacobian;
   private int numJoints;

   private YoDouble[] originalDesiredPositions;
   private YoDouble[] alphas;
   private YoDouble[] comparisonValues;
   private YoDouble[] adjustedDesiredPositions;
   private YoDouble[] lowerLimits;
   private YoDouble[] upperLimits;
   private YoFramePose originalDesiredYoPose;
   private FramePose3D originalDesiredPose;
   private FramePoint3D adjustedDesiredPosition;
   private FrameQuaternion adjustedDesiredOrientation;
   private YoFramePose adjustedDesiredPose;
   private RigidBodyTransform desiredTransform;
   private YoFrameVector originalDesiredLinearVelocity;
   private YoFrameVector adjustedDesiredLinearVelocity;

   private final LinearSolver<DenseMatrix64F> solver;
   private final DenseMatrix64F jacobianMatrix;
   private final DenseMatrix64F jacobianMatrixTransposed;
   private final DenseMatrix64F jacobianTimesJaconianTransposedMatrix;
   private final DenseMatrix64F lamdaSquaredMatrix;
   private final DenseMatrix64F jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix;
   private final DenseMatrix64F translationSelectionMatrix;
   private final DenseMatrix64F allSelectionMatrix;
   private final DenseMatrix64F originalDesiredVelocity;
   private final DenseMatrix64F intermediateResult;
   private final DenseMatrix64F jointVelocities;
   private final DenseMatrix64F adjustedDesiredVelocity;

   private final YoGraphicPosition yoDesiredFootPositionGraphic, yoCorrectedDesiredFootPositionGraphic;

   public LegJointLimitAvoidanceControlModule(String prefix, YoVariableRegistry registry, HighLevelHumanoidControllerToolbox controllerToolbox, RobotSide robotSide)
   {
      robotModel = controllerToolbox.getFullRobotModel();
      base = robotModel.getPelvis();
      RigidBody foot = robotModel.getFoot(robotSide);
      robotJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(base, foot), OneDoFJoint.class);
      ikJoints = ScrewTools.filterJoints(ScrewTools.cloneJointPath(robotJoints), OneDoFJoint.class);
      jacobian = new GeometricJacobian(ikJoints, ikJoints[ikJoints.length - 1].getSuccessor().getBodyFixedFrame());

      inverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterationsForIK, maxStepSize,
            minRandomSearchScalar, maxRandomSearchScalar);
      inverseKinematicsCalculator.setLimitJointAngles(true);

      numJoints = ikJoints.length;
      {
         originalDesiredPositions = new YoDouble[numJoints];
         alphas = new YoDouble[numJoints];
         comparisonValues = new YoDouble[numJoints];
         adjustedDesiredPositions = new YoDouble[numJoints];
         lowerLimits = new YoDouble[numJoints];
         upperLimits = new YoDouble[numJoints];

         for (int i = 0; i < numJoints; i++)
         {
            originalDesiredPositions[i] = new YoDouble(prefix + "originalDesiredPositions" + i, registry);
            alphas[i] = new YoDouble(prefix + "alpha" + i, registry);
            comparisonValues[i] = new YoDouble(prefix + "comparisonValues" + i, registry);
            adjustedDesiredPositions[i] = new YoDouble(prefix + "adjustedDesiredPositions" + i, registry);
            lowerLimits[i] = new YoDouble(prefix + "lowerLimits" + i, registry);
            upperLimits[i] = new YoDouble(prefix + "upperLimits" + i, registry);
         }

         originalDesiredPose = new FramePose3D();
         originalDesiredYoPose = new YoFramePose(prefix + "originalDesiredYoPose", ReferenceFrame.getWorldFrame(), registry);
         adjustedDesiredPose = new YoFramePose(prefix + "adjustedDesiredPose", ReferenceFrame.getWorldFrame(), registry);
         desiredTransform = new RigidBodyTransform();
         adjustedDesiredPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
         adjustedDesiredOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      }

      percentJointRangeForThreshold = new YoDouble(prefix + "percentJointRangeForThreshold", registry);
      percentJointRangeForThreshold.set(0.5);

      jacobianMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, numJoints);
      jacobianMatrixTransposed = new DenseMatrix64F(numJoints, SpatialMotionVector.SIZE);
      jacobianTimesJaconianTransposedMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);

      lamdaSquaredMatrix = new DenseMatrix64F(numJoints, numJoints);
      jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix = new DenseMatrix64F(numJoints, numJoints);

      solver = LinearSolverFactory.leastSquares(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      translationSelectionMatrix = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
      translationSelectionMatrix.set(0, 3, 1.0);
      translationSelectionMatrix.set(1, 4, 1.0);
      translationSelectionMatrix.set(2, 5, 1.0);

      allSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      allSelectionMatrix.set(5, 5, 1.0);

      originalDesiredVelocity = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
      intermediateResult = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

      jointVelocities = new DenseMatrix64F(numJoints, 1);
      adjustedDesiredVelocity = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);

      originalDesiredLinearVelocity = new YoFrameVector(prefix + "originalDesiredLinearVelocity", ReferenceFrame.getWorldFrame(), registry);
      adjustedDesiredLinearVelocity = new YoFrameVector(prefix + "adjustedDesiredLinearVelocity", ReferenceFrame.getWorldFrame(), registry);

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      if (visualize)
      {
         yoDesiredFootPositionGraphic = new YoGraphicPosition(prefix + "DesiredFootPosition", originalDesiredYoPose.getPosition(), 0.025, YoAppearance.Yellow(),
               YoGraphicPosition.GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoDesiredFootPositionGraphic);
         yoCorrectedDesiredFootPositionGraphic = new YoGraphicPosition(prefix + "CorrectedDesiredFootPosition", adjustedDesiredPose.getPosition(), 0.025,
               YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL);
         yoGraphicsListRegistry.registerYoGraphic("SingularityCollapseAvoidance", yoCorrectedDesiredFootPositionGraphic);
      }
      else
      {
         yoDesiredFootPositionGraphic = null;
         yoCorrectedDesiredFootPositionGraphic = null;
      }
   }

   public void correctSwingFootTrajectory(FramePoint3D desiredPosition, FrameQuaternion desiredOrientation, FrameVector3D desiredLinearVelocityOfOrigin,
         FrameVector3D desiredAngularVelocity, FrameVector3D desiredLinearAccelerationOfOrigin, FrameVector3D desiredAngularAcceleration)

   {
      // update joint positions in the ikJoints to the current positions
      updateJointPositions();

      Twist rootJointTist = new Twist();
      robotModel.getRootJoint().getJointTwist(rootJointTist);
      FrameVector3D linearRootJointVelocity = new FrameVector3D();
      rootJointTist.getLinearPart(linearRootJointVelocity);

      linearRootJointVelocity.scale(0.004);

      originalDesiredPose.set(desiredPosition, desiredOrientation);
      originalDesiredYoPose.set(originalDesiredPose);
      originalDesiredPose.changeFrame(jacobian.getBaseFrame());

      //    originalDesiredPose.translate(linearRootJointVelocity.getX(), linearRootJointVelocity.getY(), linearRootJointVelocity.getZ());
      originalDesiredPose.get(desiredTransform);
      originalDesiredPose.changeFrame(ReferenceFrame.getWorldFrame());

      //    if (translationFixOnly){
      //       inverseKinematicsCalculator.setSelectionMatrix(translationSelectionMatrix);
      //    }else{
      //       inverseKinematicsCalculator.setSelectionMatrix(allSelectionMatrix);
      //    }
      inverseKinematicsCalculator.solve(desiredTransform); // sets the qs of the one-dof ikJoints

      //    adjust joints based on joint angle limits
      adjustJointPositions();

      // calculate the new desired position and orientation
      ikJoints[0].updateFramesRecursively();
      RigidBodyTransform newFootTransform = jacobian.getEndEffectorFrame().getTransformToDesiredFrame(jacobian.getBaseFrame());
      RigidBodyTransform footInWorldTransform = jacobian.getEndEffectorFrame().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      ReferenceFrame adjustedFootFrame = jacobian.getEndEffectorFrame();

      adjustedDesiredPosition.setToZero(adjustedFootFrame);
      adjustedDesiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      adjustedDesiredOrientation.setToZero(adjustedFootFrame);
      adjustedDesiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      adjustedDesiredPose.setPosition(adjustedDesiredPosition);

      if (!translationFixOnly)
      {
         adjustedDesiredPose.setOrientation(adjustedDesiredOrientation);
      }

      if (enableCorrection)
      {
         desiredPosition.set(adjustedDesiredPosition);

         if (!translationFixOnly)
         {
            desiredOrientation.set(adjustedDesiredOrientation);
         }
      }

      // calculate the adjusted joint velocities using the alphas, then calculate the adjusted velocities
      desiredAngularVelocity.get(0, originalDesiredVelocity);
      desiredLinearVelocityOfOrigin.get(3, originalDesiredVelocity);
      calculateAdjustedVelocities();
      double[] adjustedVelocities = adjustedDesiredVelocity.getData();

      //    if (enableCorrection)
      {
         if (!translationFixOnly)
         {
            desiredAngularVelocity.set(adjustedVelocities[0], adjustedVelocities[1], adjustedVelocities[2]);
         }

         desiredLinearVelocityOfOrigin.set(adjustedVelocities[3], adjustedVelocities[4], adjustedVelocities[5]);
      }

      if (visualize)
      {
         yoDesiredFootPositionGraphic.showGraphicObject();
         yoCorrectedDesiredFootPositionGraphic.showGraphicObject();
      }
   }

   private void updateJointPositions()
   {
      for (int i = 0; i < numJoints; i++)
      {
         ikJoints[i].setQ(robotJoints[i].getQ());
      }
   }

   private void adjustJointPositions()
   {
      int size = ikJoints.length;
      double upperLimit;
      double lowerLimit;
      double midpointOfLimits;
      double range;
      double rangePercentageForThreshold = percentJointRangeForThreshold.getDoubleValue();
      double lambda = 1 - rangePercentageForThreshold;

      for (int i = 0; i < size; i++)
      {
         lowerLimit = ikJoints[i].getJointLimitLower();
         upperLimit = ikJoints[i].getJointLimitUpper();
         lowerLimits[i].set(lowerLimit);
         upperLimits[i].set(upperLimit);
         midpointOfLimits = (lowerLimit + upperLimit) / 2;
         range = upperLimit - lowerLimit;

         originalDesiredPositions[i].set(ikJoints[i].getQ());
         double adjustedPosition = originalDesiredPositions[i].getDoubleValue();

         double comparisonValue = (2 * (originalDesiredPositions[i].getDoubleValue() - midpointOfLimits) / range); // should range between -1 and 1, which are the limits
         comparisonValue = Math.min(comparisonValue, 1.0);
         comparisonValue = Math.max(comparisonValue, -1.0);

         double alpha = 0;
         if ((comparisonValue > lambda) || (comparisonValue < -lambda))
         {
            alpha = Math.max(0.0, (Math.abs(comparisonValue) - lambda) / rangePercentageForThreshold);
         }

         alphas[i].set(alpha);

         adjustedPosition = (1.0 - alpha) * ikJoints[i].getQ() + alpha * robotJoints[i].getQ();

         adjustedPosition = Math.max(lowerLimit, adjustedPosition);
         adjustedPosition = Math.min(upperLimit, adjustedPosition);
         adjustedDesiredPositions[i].set(adjustedPosition);
         ikJoints[i].setQ(adjustedPosition);
      }
   }

   private void calculateAdjustedVelocities()
   {
      int numberOfConstraints = SpatialMotionVector.SIZE;

      updateJointPositions();
      jacobian.compute();

      // J
      jacobianMatrix.set(jacobian.getJacobianMatrix());

      // J^T
      CommonOps.transpose(jacobianMatrix, jacobianMatrixTransposed);

      // J J^T
      CommonOps.multOuter(jacobianMatrix, jacobianTimesJaconianTransposedMatrix);

      intermediateResult.reshape(numberOfConstraints, 1);

      lamdaSquaredMatrix.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps.setIdentity(lamdaSquaredMatrix);
      lamdaSquaredMatrix.zero();

      //    CommonOps.scale(lambdaLeastSquares, lamdaSquaredMatrix);

      jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix.reshape(numberOfConstraints, numberOfConstraints);
      jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix.set(jacobianTimesJaconianTransposedMatrix);
      CommonOps.add(jacobianTimesJaconianTransposedMatrix, lamdaSquaredMatrix, jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix);

      boolean success = solver.setA(jacobianTimesJaconianTransposedPlusLamdaSquaredMatrix);

      // Solve J*J^T deltaX = f
      solver.solve(originalDesiredVelocity, intermediateResult);
      CommonOps.mult(jacobianMatrixTransposed, intermediateResult, jointVelocities);

      for (int i = 0; i < numJoints; i++)
      {
         if (comparisonValues[i].getDoubleValue() * jointVelocities.get(i) > 0)
         {
            jointVelocities.times(i, (1 - alphas[i].getDoubleValue()));
         }
      }

      CommonOps.mult(jacobianMatrix, jointVelocities, adjustedDesiredVelocity);
   }
}
