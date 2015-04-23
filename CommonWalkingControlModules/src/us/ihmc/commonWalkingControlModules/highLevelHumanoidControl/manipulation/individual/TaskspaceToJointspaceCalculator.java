package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.utilities.kinematics.InverseJacobianSolver;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TaskspaceToJointspaceCalculator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable lambdaLeastSquares;

   private final FramePose desiredControlFramePose = new FramePose();
   private final Twist desiredControlFrameTwist = new Twist();

   private final OneDoFJoint[] localJoints;
   private final ReferenceFrame baseFrame;
   private final ReferenceFrame localEndEffectorFrame;
   private final ReferenceFrame originalEndEffectorFrame;
   private final ReferenceFrame localControlFrame;
   private ReferenceFrame originalControlFrame;

   private final GeometricJacobian jacobian;
   private final InverseJacobianSolver inverseJacobianSolver;
   private final int numberOfDoF;
   private final int maxNumberOfConstraints = SpatialMotionVector.SIZE;

   private final DenseMatrix64F jointAngleCorrections;
   private final DenseMatrix64F maximumAngleCorrections;
   private final DenseMatrix64F maximumJointVelocities;

   private final AxisAngle4d errorAxisAngle = new AxisAngle4d();
   private final Vector3d errorRotationVector = new Vector3d();
   private final Vector3d errorTranslationVector = new Vector3d();
   private final DenseMatrix64F spatialError = new DenseMatrix64F(maxNumberOfConstraints, 1);
   private final DenseMatrix64F subspaceSpatialError = new DenseMatrix64F(maxNumberOfConstraints, 1);
   private final DenseMatrix64F spatialDesiredVelocity = new DenseMatrix64F(maxNumberOfConstraints, 1);

   private final double jointAngleRegularizationWeight = 1.0;
   private final DenseMatrix64F privilegedJointPositions;
   private final DenseMatrix64F privilegedJointVelocities;
   private final DenseMatrix64F jointSquaredRangeOfMotions;
   private final DenseMatrix64F desiredJointAngles;
   private final DenseMatrix64F desiredJointVelocities;

   private final double controlDT;

   public TaskspaceToJointspaceCalculator(String namePrefix, RigidBody base, RigidBody endEffector, double controlDT, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, base, endEffector, controlDT, false, parentRegistry);
   }

   public TaskspaceToJointspaceCalculator(String namePrefix, RigidBody base, RigidBody endEffector, double controlDT, boolean useSVD, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;

      localJoints = ScrewTools.cloneOneDoFJointPath(base, endEffector);
      numberOfDoF = localJoints.length;

      baseFrame = base.getBodyFixedFrame();

      originalEndEffectorFrame = endEffector.getBodyFixedFrame();
      originalControlFrame = endEffector.getBodyFixedFrame();

      localEndEffectorFrame = localJoints[numberOfDoF - 1].getSuccessor().getBodyFixedFrame();
      localControlFrame = createLocalControlFrame(localEndEffectorFrame, originalEndEffectorFrame);

      jacobian = new GeometricJacobian(localJoints, localEndEffectorFrame);
      inverseJacobianSolver = new InverseJacobianSolver(maxNumberOfConstraints, numberOfDoF, useSVD);

      jointAngleCorrections = new DenseMatrix64F(numberOfDoF, 1);
      privilegedJointPositions = new DenseMatrix64F(numberOfDoF, 1);
      privilegedJointVelocities = new DenseMatrix64F(numberOfDoF, 1);
      desiredJointAngles = new DenseMatrix64F(numberOfDoF, 1);
      desiredJointVelocities = new DenseMatrix64F(numberOfDoF, 1);
      jointSquaredRangeOfMotions = new DenseMatrix64F(numberOfDoF, 1);
      maximumAngleCorrections = new DenseMatrix64F(numberOfDoF, 1);
      maximumJointVelocities = new DenseMatrix64F(numberOfDoF, 1);
      for (int i = 0; i < numberOfDoF; i++)
      {
         jointSquaredRangeOfMotions.set(i, 0, MathTools.square(localJoints[i].getJointLimitUpper() - localJoints[i].getJointLimitLower()));
         maximumAngleCorrections.set(i, 0, Double.POSITIVE_INFINITY);
         maximumJointVelocities.set(i, 0, Double.POSITIVE_INFINITY);
      }

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      lambdaLeastSquares = new DoubleYoVariable(namePrefix + "LambdaLeastSquares", registry);
      lambdaLeastSquares.set(0.00003);
   }

   private ReferenceFrame createLocalControlFrame(final ReferenceFrame localEndEffectorFrame, final ReferenceFrame originalEndEffectorFrame)
   {
      ReferenceFrame localControlFrame = new ReferenceFrame("controlFrame", localEndEffectorFrame)
      {
         private static final long serialVersionUID = -7254192469546194133L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            originalControlFrame.getTransformToDesiredFrame(transformToParent, originalEndEffectorFrame);
         }
      };
      return localControlFrame;
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      inverseJacobianSolver.setSelectionMatrix(selectionMatrix);
   }

   public void setFullyConstrained()
   {
      inverseJacobianSolver.setSelectionMatrixForFullConstraint();
   }

   public void setControlFrameFixedInEndEffector(ReferenceFrame controlFrame)
   {
      originalControlFrame = controlFrame;
      localControlFrame.update();
      jacobian.changeFrame(localControlFrame);
   }

   public void initialize(DenseMatrix64F jointAngles)
   {
      ScrewTools.setJointPositions(localJoints, jointAngles);
      localJoints[0].updateFramesRecursively();
   }

   public void setPrivilegedJointPositionsToMidRange()
   {
      for (int i = 0; i < numberOfDoF; i++)
      {
         double midRange = 0.5 * (localJoints[i].getJointLimitLower() + localJoints[i].getJointLimitUpper());
         privilegedJointPositions.set(i, 0, midRange);
      }
   }

   public void setMaximumJointVelocities(double maximumJointVelocities)
   {
      for (int i = 0; i < numberOfDoF; i++)
         this.maximumJointVelocities.set(i, 0, maximumJointVelocities);
      CommonOps.scale(controlDT, this.maximumJointVelocities, maximumAngleCorrections);
   }

   public void setMaximumJointAngleCorrections(double maximumJointAngleCorrections)
   {
      for (int i = 0; i < numberOfDoF; i++)
      {
         maximumAngleCorrections.set(i, 0, maximumJointAngleCorrections);
      }
      CommonOps.scale(1.0 / controlDT, maximumAngleCorrections, maximumJointVelocities);
   }

   public void setMaximumJointVelocities(DenseMatrix64F maximumJointVelocities)
   {
      this.maximumJointVelocities.set(maximumJointVelocities);
      CommonOps.scale(controlDT, this.maximumJointVelocities, maximumAngleCorrections);
   }

   public void setMaximumJointAngleCorrections(DenseMatrix64F maximumJointAngleCorrections)
   {
      CommonOps.scale(1.0 / controlDT, maximumJointAngleCorrections, maximumJointVelocities);
      maximumAngleCorrections.set(maximumJointAngleCorrections);
   }

   public void compute(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocity, FrameVector desiredAngularVelocity)
   {
      desiredControlFramePose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      desiredControlFrameTwist.set(originalEndEffectorFrame, baseFrame, originalControlFrame, desiredLinearVelocity, desiredAngularVelocity);

      compute(desiredControlFramePose, desiredControlFrameTwist);
   }

   public void compute(FramePoint desiredPosition, FrameOrientation desiredOrientation)
   {
      desiredControlFramePose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      compute(desiredControlFramePose);
   }

   public void compute(FramePose desiredPose)
   {
      jacobian.compute();
      desiredControlFramePose.setPoseIncludingFrame(desiredPose);
      computeJointAngles(desiredControlFramePose);
   }

   public void compute(Twist desiredTwist)
   {
      jacobian.compute();

      desiredControlFrameTwist.set(desiredTwist);
      computeJointVelocities(desiredControlFrameTwist);
   }

   public void compute(FramePose desiredPose, Twist desiredTwist)
   {
      jacobian.compute();

      desiredControlFramePose.setPoseIncludingFrame(desiredPose);
      desiredControlFrameTwist.set(desiredTwist);
      computeJointAnglesAndVelocities(desiredControlFramePose, desiredControlFrameTwist);
   }

   private void computeJointAngles(FramePose desiredControlFramePose)
   {
      desiredControlFramePose.changeFrame(localControlFrame);

      desiredControlFramePose.getOrientation(errorAxisAngle);
      errorRotationVector.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.scale(errorAxisAngle.getAngle());

      desiredControlFramePose.getPosition(errorTranslationVector);

      MatrixTools.setDenseMatrixFromTuple3d(spatialError, errorRotationVector, 0, 0);
      MatrixTools.setDenseMatrixFromTuple3d(spatialError, errorTranslationVector, 3, 0);

      computePrivilegedJointVelocitiesForPriviligedJointAngles(privilegedJointVelocities, jointAngleRegularizationWeight, privilegedJointPositions);

      //      inverseJacobianSolver.solveUsingDampedLeastSquares(spatialError, jacobian.getJacobianMatrix(), lambdaLeastSquares.getDoubleValue());
      inverseJacobianSolver.solveUsingNullspaceMethod(spatialError, jacobian.getJacobianMatrix(), privilegedJointVelocities);
      jointAngleCorrections.set(inverseJacobianSolver.getJointspaceVelocity());

      subspaceSpatialError.reshape(inverseJacobianSolver.getNumberOfConstraints(), 1);
      subspaceSpatialError.set(inverseJacobianSolver.getSubspaceSpatialVelocity());

      for (int i = 0; i < numberOfDoF; i++)
      {
         OneDoFJoint joint = localJoints[i];
         double dq = jointAngleCorrections.get(i, 0);
         dq = MathTools.clipToMinMax(dq, maximumAngleCorrections.get(i, 0));
         double newJointAngle = joint.getQ() + dq;
         newJointAngle = MathTools.clipToMinMax(newJointAngle, joint.getJointLimitLower(), joint.getJointLimitUpper());
         joint.setQ(newJointAngle);
         desiredJointAngles.set(i, newJointAngle);
      }
   }

   private void computeJointVelocities(Twist desiredControlFrameTwist)
   {
      desiredControlFrameTwist.checkReferenceFramesMatch(originalEndEffectorFrame, baseFrame, originalControlFrame);

      desiredControlFrameTwist.packMatrix(spatialDesiredVelocity, 0);

      computePrivilegedJointVelocitiesForPriviligedJointAngles(privilegedJointVelocities, jointAngleRegularizationWeight, privilegedJointPositions);

//    inverseJacobianSolver.solveUsingDampedLeastSquares(spatialDesiredVelocity, jacobian.getJacobianMatrix(), lambdaLeastSquares.getDoubleValue());
      inverseJacobianSolver.solveUsingNullspaceMethod(spatialDesiredVelocity, jacobian.getJacobianMatrix(), privilegedJointVelocities);
      desiredJointVelocities.set(inverseJacobianSolver.getJointspaceVelocity());

      for (int i = 0; i < numberOfDoF; i++)
      {
         desiredJointVelocities.set(i, 0, MathTools.clipToMinMax(desiredJointVelocities.get(i, 0), maximumJointVelocities.get(i, 0)));
         OneDoFJoint joint = localJoints[i];
         double qDotDesired = desiredJointVelocities.get(i, 0);
         joint.setQd(qDotDesired);
      }
   }

   private void computeJointAnglesAndVelocities(FramePose desiredControlFramePose, Twist desiredControlFrameTwist)
   {
      desiredControlFrameTwist.checkReferenceFramesMatch(originalEndEffectorFrame, baseFrame, originalControlFrame);

      desiredControlFramePose.changeFrame(localControlFrame);

      desiredControlFramePose.getOrientation(errorAxisAngle);
      errorRotationVector.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.scale(errorAxisAngle.getAngle());

      desiredControlFramePose.getPosition(errorTranslationVector);

      MatrixTools.setDenseMatrixFromTuple3d(spatialError, errorRotationVector, 0, 0);
      MatrixTools.setDenseMatrixFromTuple3d(spatialError, errorTranslationVector, 3, 0);
      desiredControlFrameTwist.packMatrix(spatialDesiredVelocity, 0);
      CommonOps.add(1.0 / controlDT, spatialError, spatialDesiredVelocity, spatialDesiredVelocity);

      computePrivilegedJointVelocitiesForPriviligedJointAngles(privilegedJointVelocities, jointAngleRegularizationWeight, privilegedJointPositions);

//      inverseJacobianSolver.solveUsingDampedLeastSquares(spatialDesiredVelocity, jacobian.getJacobianMatrix(), lambdaLeastSquares.getDoubleValue());
      inverseJacobianSolver.solveUsingNullspaceMethod(spatialDesiredVelocity, jacobian.getJacobianMatrix(), privilegedJointVelocities);
      desiredJointVelocities.set(inverseJacobianSolver.getJointspaceVelocity());

      subspaceSpatialError.reshape(inverseJacobianSolver.getNumberOfConstraints(), 1);
      subspaceSpatialError.set(inverseJacobianSolver.getSubspaceSpatialVelocity());
      CommonOps.scale(controlDT, subspaceSpatialError);

      for (int i = 0; i < numberOfDoF; i++)
      {
         OneDoFJoint joint = localJoints[i];
         double qDotDesired = desiredJointVelocities.get(i, 0);
         double dq = qDotDesired * controlDT;
         jointAngleCorrections.set(i, 0, dq);
         dq = MathTools.clipToMinMax(dq, maximumAngleCorrections.get(i, 0));
         double newJointAngle = joint.getQ() + dq;
         newJointAngle = MathTools.clipToMinMax(newJointAngle, joint.getJointLimitLower(), joint.getJointLimitUpper());
         desiredJointVelocities.set(i, 0, (newJointAngle - joint.getQ()) / controlDT);
         joint.setQ(newJointAngle);
         joint.setQd(qDotDesired);
         desiredJointAngles.set(i, newJointAngle);
         joint.getFrameAfterJoint().update();
      }
   }

   private void computePrivilegedJointVelocitiesForPriviligedJointAngles(DenseMatrix64F privilegedJointVelocitiesToPack, double weight, DenseMatrix64F privilegedJointPositions)
   {
      for (int i = 0; i < numberOfDoF; i++)
      {
         privilegedJointVelocitiesToPack.set(i, 0, - 2.0 * weight * (localJoints[i].getQ() - privilegedJointPositions.get(i)) / jointSquaredRangeOfMotions.get(i, 0));
      }
   }
 
   public ReferenceFrame getControlFrame()
   {
      return originalControlFrame;
   }

   public DenseMatrix64F getSubspaceSpatialError()
   {
      return subspaceSpatialError;
   }

   public double getSpatialErrorScalar()
   {
      return NormOps.normP2(subspaceSpatialError);
   }

   public DenseMatrix64F getDesiredJointAngles()
   {
      return desiredJointAngles;
   }

   public DenseMatrix64F getDesiredJointVelocities()
   {
      return desiredJointVelocities;
   }

   public void packDesiredJointAnglesIntoOneDoFJoints(OneDoFJoint[] joints)
   {
      ScrewTools.setDesiredJointPositions(joints, desiredJointAngles);
   }

   public void packDesiredJointVelocitiesIntoOneDoFJoints(OneDoFJoint[] joints)
   {
      ScrewTools.setDesiredJointVelocities(joints, desiredJointVelocities);
   }
}
