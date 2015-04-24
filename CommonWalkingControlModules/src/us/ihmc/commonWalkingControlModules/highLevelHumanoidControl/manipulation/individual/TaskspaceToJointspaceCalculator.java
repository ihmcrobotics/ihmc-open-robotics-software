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
import us.ihmc.yoUtilities.math.YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class TaskspaceToJointspaceCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final FramePose desiredControlFramePose = new FramePose();
   private final Twist desiredControlFrameTwist = new Twist();

   private final OneDoFJoint[] localJoints;
   private final ReferenceFrame baseFrame;
   private final ReferenceFrame localEndEffectorFrame;
   private final ReferenceFrame originalEndEffectorFrame;
   private final ReferenceFrame localControlFrame;
   private ReferenceFrame originalControlFrame;

   private final GeometricJacobian jacobian;
   private final YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities solver;
   private final InverseJacobianSolver inverseJacobianSolver;
   private final int numberOfDoF;
   private final int maxNumberOfConstraints = SpatialMotionVector.SIZE;

   private final DenseMatrix64F maximumJointVelocities;
   private final DenseMatrix64F maximumJointAccelerations;

   private final DoubleYoVariable maximumTaskspaceAngularVelocityMagnitude;
   private final DoubleYoVariable maximumTaskspaceLinearVelocityMagnitude;

   private final YoFrameVector yoAngularVelocityFromError;
   private final YoFrameVector yoLinearVelocityFromError;

   private final AxisAngle4d errorAxisAngle = new AxisAngle4d();
   private final Vector3d errorRotationVector = new Vector3d();
   private final Vector3d errorTranslationVector = new Vector3d();
   private final DenseMatrix64F spatialVelocityFromError = new DenseMatrix64F(maxNumberOfConstraints, 1);
   private final DenseMatrix64F subspaceSpatialError = new DenseMatrix64F(maxNumberOfConstraints, 1);
   private final DenseMatrix64F spatialDesiredVelocity = new DenseMatrix64F(maxNumberOfConstraints, 1);

   private final double jointAngleRegularizationWeight = 1.0;
   private final DenseMatrix64F privilegedJointPositions;
   private final DenseMatrix64F privilegedJointVelocities;
   private final DenseMatrix64F jointSquaredRangeOfMotions;
   private final DenseMatrix64F desiredJointAngles;
   private final DenseMatrix64F desiredJointVelocities;
   private final DenseMatrix64F desiredJointAccelerations;

   private final double controlDT;

   public TaskspaceToJointspaceCalculator(String namePrefix, RigidBody base, RigidBody endEffector, double controlDT, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.controlDT = controlDT;

      localJoints = ScrewTools.cloneOneDoFJointPath(base, endEffector);
      numberOfDoF = localJoints.length;

      baseFrame = base.getBodyFixedFrame();

      originalEndEffectorFrame = endEffector.getBodyFixedFrame();
      originalControlFrame = endEffector.getBodyFixedFrame();

      localEndEffectorFrame = localJoints[numberOfDoF - 1].getSuccessor().getBodyFixedFrame();
      localControlFrame = createLocalControlFrame(localEndEffectorFrame, originalEndEffectorFrame);

      jacobian = new GeometricJacobian(localJoints, localEndEffectorFrame);
      solver = new YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities(namePrefix, maxNumberOfConstraints, maxNumberOfConstraints, registry);

      inverseJacobianSolver = new InverseJacobianSolver(maxNumberOfConstraints, numberOfDoF, solver);

      privilegedJointPositions = new DenseMatrix64F(numberOfDoF, 1);
      privilegedJointVelocities = new DenseMatrix64F(numberOfDoF, 1);

      desiredJointAngles = new DenseMatrix64F(numberOfDoF, 1);
      desiredJointVelocities = new DenseMatrix64F(numberOfDoF, 1);
      desiredJointAccelerations = new DenseMatrix64F(numberOfDoF, 1);

      jointSquaredRangeOfMotions = new DenseMatrix64F(numberOfDoF, 1);

      maximumJointVelocities = new DenseMatrix64F(numberOfDoF, 1);
      maximumJointAccelerations = new DenseMatrix64F(numberOfDoF, 1);

      maximumTaskspaceAngularVelocityMagnitude = new DoubleYoVariable(namePrefix + "MaximumTaskspaceAngularVelocityMagnitude", registry);
      maximumTaskspaceLinearVelocityMagnitude = new DoubleYoVariable(namePrefix + "MaximumTaskspaceLinearVelocityMagnitude", registry);

      maximumTaskspaceAngularVelocityMagnitude.set(Double.POSITIVE_INFINITY);
      maximumTaskspaceLinearVelocityMagnitude.set(Double.POSITIVE_INFINITY);

      for (int i = 0; i < numberOfDoF; i++)
      {
         jointSquaredRangeOfMotions.set(i, 0, MathTools.square(localJoints[i].getJointLimitUpper() - localJoints[i].getJointLimitLower()));
         maximumJointVelocities.set(i, 0, Double.POSITIVE_INFINITY);
         maximumJointAccelerations.set(i, 0, Double.POSITIVE_INFINITY);
      }

      yoAngularVelocityFromError = new YoFrameVector(namePrefix + "AngularVelocityFromError", worldFrame, registry);
      yoLinearVelocityFromError = new YoFrameVector(namePrefix + "LinearVelocityFromError", worldFrame, registry);

      parentRegistry.addChild(registry);
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
   }

   public void setMaximumJointAngleCorrections(double maximumJointAngleCorrections)
   {
      for (int i = 0; i < numberOfDoF; i++)
         this.maximumJointVelocities.set(i, 0, maximumJointAngleCorrections / controlDT);
   }
   
   public void setMaximumJointAccelerations(double maximumJointAccelerations)
   {
      for (int i = 0; i < numberOfDoF; i++)
         this.maximumJointAccelerations.set(i, 0, maximumJointAccelerations);
   }

   public void setMaximumJointVelocities(DenseMatrix64F maximumJointVelocities)
   {
      this.maximumJointVelocities.set(maximumJointVelocities);
   }

   public void setMaximumJointAngleCorrections(DenseMatrix64F maximumJointAngleCorrections)
   {
      CommonOps.scale(1.0 / controlDT, maximumJointAngleCorrections, maximumJointVelocities);
   }

   public void setMaximumJointAccelerations(DenseMatrix64F maximumJointAccelerations)
   {
      this.maximumJointAccelerations.set(maximumJointAccelerations);
   }

   public void setMaximumTaskspaceVelocity(double maximumAngularVelocity, double maximumLinearVelocity)
   {
      this.maximumTaskspaceAngularVelocityMagnitude.set(maximumAngularVelocity);
      this.maximumTaskspaceLinearVelocityMagnitude.set(maximumLinearVelocity);
   }

   public void compute(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocity, FrameVector desiredAngularVelocity)
   {
      desiredControlFramePose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      desiredControlFrameTwist.set(originalEndEffectorFrame, baseFrame, originalControlFrame, desiredLinearVelocity, desiredAngularVelocity);

      compute(desiredControlFramePose, desiredControlFrameTwist);
   }

   public void compute(FramePose desiredPose, Twist desiredTwist)
   {
      jacobian.compute();

      desiredControlFramePose.setPoseIncludingFrame(desiredPose);
      desiredControlFrameTwist.set(desiredTwist);
      computeJointAnglesAndVelocities(desiredControlFramePose, desiredControlFrameTwist);
   }

   private void computeJointAnglesAndVelocities(FramePose desiredControlFramePose, Twist desiredControlFrameTwist)
   {
      desiredControlFrameTwist.checkReferenceFramesMatch(originalEndEffectorFrame, baseFrame, originalControlFrame);

      desiredControlFramePose.changeFrame(localControlFrame);

      desiredControlFramePose.getOrientation(errorAxisAngle);
      errorRotationVector.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.scale(errorAxisAngle.getAngle());

      desiredControlFramePose.getPosition(errorTranslationVector);

      MatrixTools.setDenseMatrixFromTuple3d(spatialVelocityFromError, errorRotationVector, 0, 0);
      MatrixTools.setDenseMatrixFromTuple3d(spatialVelocityFromError, errorTranslationVector, 3, 0);

      CommonOps.scale(1.0 / controlDT, spatialVelocityFromError);

      limitateSpatialVelocityFromError(spatialVelocityFromError);

      desiredControlFrameTwist.packMatrix(spatialDesiredVelocity, 0);
      CommonOps.add(spatialVelocityFromError, spatialDesiredVelocity, spatialDesiredVelocity);

      computePrivilegedJointVelocitiesForPriviligedJointAngles(privilegedJointVelocities, jointAngleRegularizationWeight, privilegedJointPositions);

      inverseJacobianSolver.solveUsingNullspaceMethod(spatialDesiredVelocity, jacobian.getJacobianMatrix(), privilegedJointVelocities);
      desiredJointVelocities.set(inverseJacobianSolver.getJointspaceVelocity());

      subspaceSpatialError.reshape(inverseJacobianSolver.getNumberOfConstraints(), 1);
      subspaceSpatialError.set(inverseJacobianSolver.getSubspaceSpatialVelocity());
      CommonOps.scale(controlDT, subspaceSpatialError);

      for (int i = 0; i < numberOfDoF; i++)
      {
         OneDoFJoint joint = localJoints[i];
         double qDotDesired = MathTools.clipToMinMax(desiredJointVelocities.get(i, 0), maximumJointVelocities.get(i, 0));
         double qDotDotDesired = (qDotDesired - joint.getQd()) / controlDT;
         qDotDotDesired = MathTools.clipToMinMax(qDotDotDesired, maximumJointAccelerations.get(i, 0));
         qDotDesired = joint.getQd() + qDotDotDesired * controlDT;
         
         double qDesired = joint.getQ() + qDotDesired * controlDT;
         qDesired = MathTools.clipToMinMax(qDesired, joint.getJointLimitLower(), joint.getJointLimitUpper());
         qDotDesired = (qDesired - joint.getQ()) / controlDT;
         qDotDotDesired = (qDotDesired - joint.getQd()) / controlDT;

         joint.setQ(qDesired);
         joint.setQd(qDotDesired);
         joint.setQdd(qDotDotDesired);

         desiredJointAngles.set(i, qDesired);
         desiredJointVelocities.set(i, 0, qDotDesired);
         desiredJointAccelerations.set(i, 0, qDotDotDesired);

         joint.getFrameAfterJoint().update();
      }
   }

   private final FrameVector angularVelocityFromError = new FrameVector();
   private final FrameVector linearVelocityFromError = new FrameVector();

   private void limitateSpatialVelocityFromError(DenseMatrix64F spatialVelocityFromError)
   {
      MatrixTools.extractFrameTupleFromEJMLVector(angularVelocityFromError, spatialVelocityFromError, localControlFrame, 0);
      MatrixTools.extractFrameTupleFromEJMLVector(linearVelocityFromError, spatialVelocityFromError, localControlFrame, 3);

      clipToVectorMagnitude(maximumTaskspaceAngularVelocityMagnitude.getDoubleValue(), angularVelocityFromError);
      clipToVectorMagnitude(maximumTaskspaceLinearVelocityMagnitude.getDoubleValue(), linearVelocityFromError);

      MatrixTools.insertFrameTupleIntoEJMLVector(angularVelocityFromError, spatialVelocityFromError, 0);
      MatrixTools.insertFrameTupleIntoEJMLVector(linearVelocityFromError, spatialVelocityFromError, 3);

      yoAngularVelocityFromError.setAndMatchFrame(angularVelocityFromError);
      yoLinearVelocityFromError.setAndMatchFrame(linearVelocityFromError);
   }

   private void clipToVectorMagnitude(double maximumMagnitude, FrameVector frameVectorToClip)
   {
      double magnitude = frameVectorToClip.length();
      if (magnitude > maximumMagnitude)
      {
         frameVectorToClip.scale(maximumMagnitude / magnitude);
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

   public DenseMatrix64F getDesiredJointAccelerations()
   {
      return desiredJointAccelerations;
   }

   public void packDesiredJointAnglesIntoOneDoFJoints(OneDoFJoint[] joints)
   {
      ScrewTools.setDesiredJointPositions(joints, desiredJointAngles);
   }

   public void packDesiredJointVelocitiesIntoOneDoFJoints(OneDoFJoint[] joints)
   {
      ScrewTools.setDesiredJointVelocities(joints, desiredJointVelocities);
   }

   public double computeDeterminant()
   {
      jacobian.compute();
      return inverseJacobianSolver.computeDeterminant(jacobian.getJacobianMatrix());
   }

   public double getLastComputedDeterminant()
   {
      return inverseJacobianSolver.getLastComputedDeterminant();
   }
}
