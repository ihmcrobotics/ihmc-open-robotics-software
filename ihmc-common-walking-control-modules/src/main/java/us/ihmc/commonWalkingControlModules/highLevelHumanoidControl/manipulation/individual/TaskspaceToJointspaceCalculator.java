package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.kinematics.InverseJacobianSolver;
import us.ihmc.robotics.math.YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class TaskspaceToJointspaceCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   public enum SecondaryObjective
   {
      TOWARD_RESTING_CONFIGURATION, AWAY_FROM_JOINT_LIMITS
   };

   private final RigidBodyTransform desiredControlFrameTransform = new RigidBodyTransform();
   private final FramePose3D desiredControlFramePose = new FramePose3D();
   private final Twist desiredControlFrameTwist = new Twist();

   private final OneDoFJointBasics[] originalJoints;
   private final OneDoFJointBasics[] localJoints;
   private final ReferenceFrame originalBaseFrame;
   private final ReferenceFrame localBaseFrame;
   private final ReferenceFrame originalBaseParentJointFrame;
   private final PoseReferenceFrame localBaseParentJointFrame;
   private final ReferenceFrame localEndEffectorFrame;
   private final ReferenceFrame originalEndEffectorFrame;
   private final ReferenceFrame localControlFrame;
   private ReferenceFrame originalControlFrame;

   private final Map<ReferenceFrame, ReferenceFrame> originalToLocalFramesMap = new LinkedHashMap<>();

   private final GeometricJacobian jacobian;
   private final YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities solver;
   private final InverseJacobianSolver inverseJacobianSolver;
   private final int numberOfDoF;
   private final int maxNumberOfConstraints = SpatialVector.SIZE;

   private final YoDouble jointAngleRegularizationWeight;
   private final YoInteger exponentForPNorm;
   private final YoEnum<SecondaryObjective> currentSecondaryObjective;

   private final YoDouble maximumJointVelocity;
   private final YoDouble maximumJointAcceleration;

   private final YoDouble maximumTaskspaceAngularVelocityMagnitude;
   private final YoDouble maximumTaskspaceLinearVelocityMagnitude;

   private final YoFramePoseUsingYawPitchRoll yoDesiredControlFramePose;

   private final YoFrameVector3D yoErrorRotation;
   private final YoFrameVector3D yoErrorTranslation;

   private final YoFrameVector3D yoAngularVelocityFromError;
   private final YoFrameVector3D yoLinearVelocityFromError;

   private final YoDouble alphaSpatialVelocityFromError;
   private final AlphaFilteredYoFrameVector filteredAngularVelocityFromError;
   private final AlphaFilteredYoFrameVector filteredLinearVelocityFromError;

   private final YoFramePoint3D yoBaseParentJointFramePosition;
   private final YoFrameQuaternion yoBaseParentJointFrameOrientation;

   private final YoDouble alphaBaseParentJointPose;
   private final AlphaFilteredYoFramePoint yoBaseParentJointFramePositionFiltered;
   private final AlphaFilteredYoFrameQuaternion yoBaseParentJointFrameOrientationFiltered;

   private final YoBoolean enableFeedbackControl;
   private final YoDouble kpTaskspaceAngularError;
   private final YoDouble kpTaskspaceLinearError;

   private final FramePoint3D baseParentJointFramePosition = new FramePoint3D();
   private final FrameQuaternion baseParentJointFrameOrientation = new FrameQuaternion();

   private final AxisAngle errorAxisAngle = new AxisAngle();
   private final Vector3D errorRotationVector = new Vector3D();
   private final Vector3D errorTranslationVector = new Vector3D();
   private final DMatrixRMaj spatialVelocityFromError = new DMatrixRMaj(maxNumberOfConstraints, 1);
   private final DMatrixRMaj subspaceSpatialError = new DMatrixRMaj(maxNumberOfConstraints, 1);
   private final DMatrixRMaj spatialDesiredVelocity = new DMatrixRMaj(maxNumberOfConstraints, 1);

   private final YoDouble[] yoPrivilegedJointPositions;
   private final AlphaFilteredYoVariable[] yoPrivilegedJointPositionsFiltered;

   private final DMatrixRMaj privilegedJointVelocities;
   private final DMatrixRMaj jointSquaredRangeOfMotions;
   private final DMatrixRMaj jointAnglesAtMidRangeOfMotion;
   private final DMatrixRMaj desiredJointAngles;
   private final DMatrixRMaj desiredJointVelocities;
   private final DMatrixRMaj desiredJointAccelerations;

   private final FramePose3D originalBasePose = new FramePose3D();

   private final double controlDT;

   public TaskspaceToJointspaceCalculator(String namePrefix, RigidBodyBasics base, RigidBodyBasics endEffector, double controlDT,
                                          YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.controlDT = controlDT;

      originalBaseParentJointFrame = base.getParentJoint().getFrameAfterJoint();
      originalBaseFrame = base.getBodyFixedFrame();
      localBaseParentJointFrame = new PoseReferenceFrame(base.getName() + "Local", worldFrame);
      String localBaseFrameName = originalBaseFrame.getName() + "Local";
      RigidBodyTransform transformToParent = originalBaseFrame.getTransformToDesiredFrame(originalBaseParentJointFrame);
      localBaseFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(localBaseFrameName, localBaseParentJointFrame, transformToParent);

      originalJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);
      localJoints = MultiBodySystemTools.filterJoints(MultiBodySystemFactories.cloneKinematicChain(originalJoints, "Local", localBaseParentJointFrame),
                                                      OneDoFJointBasics.class);
      numberOfDoF = localJoints.length;

      originalEndEffectorFrame = endEffector.getBodyFixedFrame();
      originalControlFrame = endEffector.getBodyFixedFrame();

      localEndEffectorFrame = localJoints[numberOfDoF - 1].getSuccessor().getBodyFixedFrame();
      localControlFrame = createLocalControlFrame(localEndEffectorFrame, originalEndEffectorFrame);

      populateRefrenceFrameMap();

      jacobian = new GeometricJacobian(localJoints, localEndEffectorFrame, true);
      solver = new YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities(namePrefix, maxNumberOfConstraints, maxNumberOfConstraints, registry);

      inverseJacobianSolver = new InverseJacobianSolver(maxNumberOfConstraints, numberOfDoF, solver);

      jointAngleRegularizationWeight = new YoDouble(namePrefix + "JointAngleRegularizationWeight", registry);

      exponentForPNorm = new YoInteger(namePrefix + "ExponentForPNorm", registry);
      exponentForPNorm.set(6);

      currentSecondaryObjective = new YoEnum<>(namePrefix + "SecondaryObjective", registry, SecondaryObjective.class);
      currentSecondaryObjective.set(SecondaryObjective.TOWARD_RESTING_CONFIGURATION);

      yoPrivilegedJointPositions = new YoDouble[numberOfDoF];
      yoPrivilegedJointPositionsFiltered = new AlphaFilteredYoVariable[numberOfDoF];
      privilegedJointVelocities = new DMatrixRMaj(numberOfDoF, 1);

      desiredJointAngles = new DMatrixRMaj(numberOfDoF, 1);
      desiredJointVelocities = new DMatrixRMaj(numberOfDoF, 1);
      desiredJointAccelerations = new DMatrixRMaj(numberOfDoF, 1);

      jointSquaredRangeOfMotions = new DMatrixRMaj(numberOfDoF, 1);
      jointAnglesAtMidRangeOfMotion = new DMatrixRMaj(numberOfDoF, 1);

      maximumJointVelocity = new YoDouble(namePrefix + "MaximumJointVelocity", registry);
      maximumJointVelocity.set(Double.POSITIVE_INFINITY);
      maximumJointAcceleration = new YoDouble(namePrefix + "MaximumJointAcceleration", registry);
      maximumJointAcceleration.set(Double.POSITIVE_INFINITY);

      maximumTaskspaceAngularVelocityMagnitude = new YoDouble(namePrefix + "MaximumTaskspaceAngularVelocityMagnitude", registry);
      maximumTaskspaceLinearVelocityMagnitude = new YoDouble(namePrefix + "MaximumTaskspaceLinearVelocityMagnitude", registry);

      maximumTaskspaceAngularVelocityMagnitude.set(Double.POSITIVE_INFINITY);
      maximumTaskspaceLinearVelocityMagnitude.set(Double.POSITIVE_INFINITY);

      for (int i = 0; i < numberOfDoF; i++)
      {
         String jointName = originalJoints[i].getName();
         yoPrivilegedJointPositions[i] = new YoDouble("q_privileged_" + jointName, registry);
         yoPrivilegedJointPositionsFiltered[i] = new AlphaFilteredYoVariable("q_privileged_filt_" + jointName, registry, 0.99, yoPrivilegedJointPositions[i]);
         jointSquaredRangeOfMotions.set(i, 0, MathTools.square(localJoints[i].getJointLimitUpper() - localJoints[i].getJointLimitLower()));
         jointAnglesAtMidRangeOfMotion.set(i, 0, 0.5 * (localJoints[i].getJointLimitUpper() + localJoints[i].getJointLimitLower()));
      }

      yoDesiredControlFramePose = new YoFramePoseUsingYawPitchRoll(namePrefix + "Desired", worldFrame, registry);

      yoErrorRotation = new YoFrameVector3D(namePrefix + "ErrorRotation", localControlFrame, registry);
      yoErrorTranslation = new YoFrameVector3D(namePrefix + "ErrorTranslation", localControlFrame, registry);

      yoAngularVelocityFromError = new YoFrameVector3D(namePrefix + "AngularVelocityFromError", localControlFrame, registry);
      yoLinearVelocityFromError = new YoFrameVector3D(namePrefix + "LinearVelocityFromError", localControlFrame, registry);

      alphaSpatialVelocityFromError = new YoDouble(namePrefix + "AlphaSpatialVelocityFromError", registry);
      filteredAngularVelocityFromError = new AlphaFilteredYoFrameVector(namePrefix
            + "FilteredAngularVelocityFromError", "", registry, alphaSpatialVelocityFromError, yoAngularVelocityFromError);
      filteredLinearVelocityFromError = new AlphaFilteredYoFrameVector(namePrefix
            + "FilteredLinearVelocityFromError", "", registry, alphaSpatialVelocityFromError, yoLinearVelocityFromError);

      yoBaseParentJointFramePosition = new YoFramePoint3D(namePrefix + "BaseParentJointFrame", worldFrame, registry);
      yoBaseParentJointFrameOrientation = new YoFrameQuaternion(namePrefix + "BaseParentJointFrame", worldFrame, registry);

      alphaBaseParentJointPose = new YoDouble(namePrefix + "AlphaBaseParentJointPose", registry);
      yoBaseParentJointFramePositionFiltered = new AlphaFilteredYoFramePoint(namePrefix
            + "BaseParentJointFramePositionFiltered", "", registry, alphaBaseParentJointPose, yoBaseParentJointFramePosition);
      yoBaseParentJointFrameOrientationFiltered = new AlphaFilteredYoFrameQuaternion(namePrefix
            + "BaseParentJointFrameOrientationFiltered", "", yoBaseParentJointFrameOrientation, alphaBaseParentJointPose, registry);

      enableFeedbackControl = new YoBoolean(namePrefix + "EnableFeedBackControl", registry);
      kpTaskspaceAngularError = new YoDouble(namePrefix + "KpTaskspaceAngularError", registry);
      kpTaskspaceLinearError = new YoDouble(namePrefix + "KpTaskspaceLinearError", registry);

      parentRegistry.addChild(registry);
   }

   private void populateRefrenceFrameMap()
   {
      originalToLocalFramesMap.put(originalBaseParentJointFrame, localBaseParentJointFrame);
      originalToLocalFramesMap.put(originalControlFrame, localControlFrame);
      originalToLocalFramesMap.put(originalBaseFrame, localBaseFrame);

      for (int i = 0; i < numberOfDoF; i++)
      {
         OneDoFJointBasics originalJoint = originalJoints[i];
         RigidBodyBasics originalBody = originalJoint.getSuccessor();
         OneDoFJointBasics localJoint = localJoints[i];
         RigidBodyBasics localBody = localJoint.getSuccessor();

         originalToLocalFramesMap.put(originalJoint.getFrameAfterJoint(), localJoint.getFrameAfterJoint());
         originalToLocalFramesMap.put(originalJoint.getFrameBeforeJoint(), localJoint.getFrameBeforeJoint());
         originalToLocalFramesMap.put(originalBody.getBodyFixedFrame(), localBody.getBodyFixedFrame());
      }
   }

   private ReferenceFrame createLocalControlFrame(final ReferenceFrame localEndEffectorFrame, final ReferenceFrame originalEndEffectorFrame)
   {
      ReferenceFrame localControlFrame = new ReferenceFrame("controlFrame", localEndEffectorFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            originalControlFrame.getTransformToDesiredFrame(transformToParent, originalEndEffectorFrame);
         }
      };
      return localControlFrame;
   }

   // TODO Need to be extracted
   public void setupWithDefaultParameters()
   {
      setFullyConstrained();
      setPrivilegedJointPositionsToMidRange();
      setJointAngleRegularizationWeight(5.0);
      setMaximumJointVelocity(5.0);
      setMaximumJointAcceleration(50.0);
      setMaximumTaskspaceVelocity(1.5, 0.5);
      setFilterBreakFrequencyForBaseFrameUpdater(1.0);
      setEnableFeedbackControl(false);
      setTaskspaceProportionalGainsForFeedbackControl(0.3, 0.3);
      setTaskspaceVelocityFromErrorFilterBreakFrequency(Double.POSITIVE_INFINITY);
   }

   public void setSelectionMatrix(DMatrixRMaj selectionMatrix)
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

   public void initialize(DMatrixRMaj jointAngles)
   {
      setLocalBaseFrameToActualAndResetFilters();
      MultiBodySystemTools.insertJointsState(localJoints, JointStateType.CONFIGURATION, jointAngles);
      localJoints[0].updateFramesRecursively();
   }

   public void initializeFromCurrentJointAngles()
   {
      setLocalBaseFrameToActualAndResetFilters();
      setLocalJointAnglesToCurrentJointAngles();
   }

   private void setLocalJointAnglesToCurrentJointAngles()
   {
      for (int i = 0; i < numberOfDoF; i++)
      {
         localJoints[i].setQ(originalJoints[i].getQ());
         localJoints[i].getFrameAfterJoint().update();
      }
   }

   public void setSecondaryObjective(SecondaryObjective secondaryObjective)
   {
      this.currentSecondaryObjective.set(secondaryObjective);
   }

   public void setJointAngleRegularizationWeight(double weight)
   {
      jointAngleRegularizationWeight.set(weight);
   }

   public void setPrivilegedJointPositionsToMidRange()
   {
      for (int i = 0; i < numberOfDoF; i++)
         yoPrivilegedJointPositions[i].set(jointAnglesAtMidRangeOfMotion.get(i, 0));
   }

   public void setPrivilegedJointPositionsToZero()
   {
      for (int i = 0; i < numberOfDoF; i++)
         yoPrivilegedJointPositions[i].set(0.0);
   }

   public void setPrivilegedJointPosition(int jointIndex, double qPrivileged)
   {
      yoPrivilegedJointPositions[jointIndex].set(qPrivileged);
   }

   public void setMaximumJointVelocity(double maximumJointVelocity)
   {
      this.maximumJointVelocity.set(maximumJointVelocity);
   }

   public void setMaximumJointAngleCorrection(double maximumJointAngleCorrection)
   {
      this.maximumJointVelocity.set(maximumJointAngleCorrection / controlDT);
   }

   public void setMaximumJointAcceleration(double maximumJointAcceleration)
   {
      this.maximumJointAcceleration.set(maximumJointAcceleration);
   }

   public void setMaximumTaskspaceVelocity(double maximumAngularVelocity, double maximumLinearVelocity)
   {
      this.maximumTaskspaceAngularVelocityMagnitude.set(maximumAngularVelocity);
      this.maximumTaskspaceLinearVelocityMagnitude.set(maximumLinearVelocity);
   }

   public void setTaskspaceVelocityFromErrorFilterBreakFrequency(double breakFrequency)
   {
      alphaSpatialVelocityFromError.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, controlDT));
   }

   public void setFilterBreakFrequencyForBaseFrameUpdater(double breakFrequency)
   {
      alphaBaseParentJointPose.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, controlDT));
   }

   public void setEnableFeedbackControl(boolean enable)
   {
      enableFeedbackControl.set(enable);
   }

   public void setTaskspaceProportionalGainsForFeedbackControl(double kPAngularError, double kPLinearError)
   {
      kpTaskspaceAngularError.set(kPAngularError);
      kpTaskspaceLinearError.set(kPLinearError);
   }

   public void compute(FramePoint3D desiredPosition, FrameQuaternion desiredOrientation, FrameVector3D desiredLinearVelocity,
                       FrameVector3D desiredAngularVelocity)
   {
      desiredControlFramePose.setIncludingFrame(desiredPosition, desiredOrientation);
      desiredControlFrameTwist.setIncludingFrame(originalEndEffectorFrame,
                                                 originalBaseFrame,
                                                 originalControlFrame,
                                                 desiredAngularVelocity,
                                                 desiredLinearVelocity);

      compute(desiredControlFramePose, desiredControlFrameTwist);
   }

   public void compute(FramePose3D desiredPose, TwistReadOnly desiredTwist)
   {
      jacobian.compute();

      desiredControlFramePose.setIncludingFrame(desiredPose);
      desiredControlFrameTwist.setIncludingFrame(desiredTwist);
      computeJointAnglesAndVelocities(desiredControlFramePose, desiredControlFrameTwist);
   }

   private final AxisAngle tempAxisAngle = new AxisAngle();

   public boolean computeIteratively(FramePose3D desiredPose, TwistReadOnly desiredTwist, double maxIterations, double epsilon)
   {
      for (int i = 0; i < maxIterations; i++)
      {
         compute(desiredPose, desiredTwist);

         tempPoint.setIncludingFrame(desiredPose.getPosition());
         tempPoint.changeFrame(localControlFrame);
         double translationDistance = tempPoint.distanceFromOrigin();

         tempOrientation.setIncludingFrame(desiredPose.getOrientation());
         tempOrientation.changeFrame(localControlFrame);
         tempAxisAngle.set(tempOrientation);
         double angle = Math.abs(AngleTools.trimAngleMinusPiToPi(tempAxisAngle.getAngle()));

         if (translationDistance < epsilon && angle < epsilon)
         {
            return true;
         }
      }

      return false;
   }

   private void computeJointAnglesAndVelocities(FramePose3D desiredControlFramePose, TwistReadOnly desiredControlFrameTwist)
   {
      if (enableFeedbackControl.getBooleanValue())
         setLocalJointAnglesToCurrentJointAngles();

      updateLocalBaseFrame();

      desiredControlFrameTwist.checkReferenceFrameMatch(originalEndEffectorFrame, originalBaseFrame, originalControlFrame);

      yoDesiredControlFramePose.setMatchingFrame(desiredControlFramePose);

      ReferenceFrame originalControlledWithRespectToFrame = desiredControlFramePose.getReferenceFrame();
      ReferenceFrame localControlledWithRespectToFrame = originalToLocalFramesMap.get(originalControlledWithRespectToFrame);
      if (localControlledWithRespectToFrame != null)
      {
         desiredControlFramePose.get(desiredControlFrameTransform);
         desiredControlFramePose.setIncludingFrame(localControlledWithRespectToFrame, desiredControlFrameTransform);
      }

      desiredControlFramePose.changeFrame(localControlFrame);

      errorAxisAngle.set(desiredControlFramePose.getOrientation());
      errorRotationVector.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.scale(errorAxisAngle.getAngle());

      errorTranslationVector.set(desiredControlFramePose.getPosition());

      yoErrorRotation.set(errorRotationVector);
      yoErrorTranslation.set(errorTranslationVector);

      if (enableFeedbackControl.getBooleanValue())
      {
         errorRotationVector.scale(kpTaskspaceAngularError.getDoubleValue());
         errorTranslationVector.scale(kpTaskspaceLinearError.getDoubleValue());
      }

      errorRotationVector.get(0, spatialVelocityFromError);
      errorTranslationVector.get(3, spatialVelocityFromError);
      CommonOps_DDRM.scale(1.0 / controlDT, spatialVelocityFromError);

      computeDesiredSpatialVelocityToSolveFor(spatialDesiredVelocity, spatialVelocityFromError, desiredControlFrameTwist);

      if (currentSecondaryObjective.getEnumValue() == SecondaryObjective.TOWARD_RESTING_CONFIGURATION)
         computePrivilegedJointVelocitiesForPriviligedJointAngles(privilegedJointVelocities, jointAngleRegularizationWeight.getDoubleValue());
      else
         computePrivilegedVelocitiesForStayingAwayFromJointLimits(privilegedJointVelocities, jointAngleRegularizationWeight.getDoubleValue());

      inverseJacobianSolver.solveUsingNullspaceMethod(spatialDesiredVelocity, jacobian.getJacobianMatrix(), privilegedJointVelocities);
      desiredJointVelocities.set(inverseJacobianSolver.getJointspaceVelocity());

      if (Double.isNaN(desiredJointVelocities.get(0)))
         throw new RuntimeException("Invalid computed desired joint velocities: " + desiredJointVelocities.toString());

      subspaceSpatialError.reshape(inverseJacobianSolver.getNumberOfConstraints(), 1);
      subspaceSpatialError.set(inverseJacobianSolver.getSubspaceSpatialVelocity());
      CommonOps_DDRM.scale(controlDT, subspaceSpatialError);

      for (int i = 0; i < numberOfDoF; i++)
      {
         OneDoFJointBasics joint = localJoints[i];
         double qDotDesired = MathTools.clamp(desiredJointVelocities.get(i, 0), maximumJointVelocity.getDoubleValue());
         double qDotDotDesired = (qDotDesired - joint.getQd()) / controlDT;
         qDotDotDesired = MathTools.clamp(qDotDotDesired, maximumJointAcceleration.getDoubleValue());
         qDotDesired = joint.getQd() + qDotDotDesired * controlDT;

         double qDesired = joint.getQ() + qDotDesired * controlDT;
         qDesired = MathTools.clamp(qDesired, joint.getJointLimitLower(), joint.getJointLimitUpper());
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

   private void updateLocalBaseFrame()
   {
      originalBasePose.setToZero(originalBaseParentJointFrame);
      originalBasePose.changeFrame(worldFrame);

      originalBasePose.get(baseParentJointFramePosition, baseParentJointFrameOrientation);
      yoBaseParentJointFrameOrientation.set(baseParentJointFrameOrientation);
      yoBaseParentJointFramePosition.set(baseParentJointFramePosition);

      yoBaseParentJointFrameOrientationFiltered.update();
      yoBaseParentJointFramePositionFiltered.update();

      baseParentJointFrameOrientation.setIncludingFrame(yoBaseParentJointFrameOrientationFiltered);
      baseParentJointFramePosition.setIncludingFrame(yoBaseParentJointFramePositionFiltered);

      localBaseParentJointFrame.setPoseAndUpdate(baseParentJointFramePosition, baseParentJointFrameOrientation);
      updateFrames();
   }

   private void setLocalBaseFrameToActualAndResetFilters()
   {
      yoBaseParentJointFrameOrientationFiltered.reset();
      yoBaseParentJointFramePositionFiltered.reset();
      updateLocalBaseFrame();
   }

   private void computeDesiredSpatialVelocityToSolveFor(DMatrixRMaj spatialDesiredVelocityToPack, DMatrixRMaj spatialVelocityFromError,
                                                        TwistReadOnly desiredControlFrameTwist)
   {
      DMatrixRMaj selectionMatrix = inverseJacobianSolver.getSelectionMatrix();
      // Clip to maximum velocity
      clipSpatialVector(spatialVelocityFromError,
                        selectionMatrix,
                        maximumTaskspaceAngularVelocityMagnitude.getDoubleValue(),
                        maximumTaskspaceLinearVelocityMagnitude.getDoubleValue());

      // Update YoVariables for the velocity
      getAngularAndLinearPartsFromSpatialVector(yoAngularVelocityFromError, yoLinearVelocityFromError, spatialVelocityFromError);

      filteredAngularVelocityFromError.update();
      filteredLinearVelocityFromError.update();

      setSpatialVectorFromAngularAndLinearParts(spatialVelocityFromError, filteredAngularVelocityFromError, filteredLinearVelocityFromError);

      desiredControlFrameTwist.get(0, spatialDesiredVelocityToPack);
      CommonOps_DDRM.add(spatialVelocityFromError, spatialDesiredVelocityToPack, spatialDesiredVelocityToPack);
   }

   private final FrameVector3D angularPart = new FrameVector3D();
   private final FrameVector3D linearPart = new FrameVector3D();
   private final DMatrixRMaj subspaceSpatialVector = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj tempSpatialVector = new DMatrixRMaj(1, 1);

   private void clipSpatialVector(DMatrixRMaj spatialVectorToClip, DMatrixRMaj selectionMatrix, double maximumAngularMagnitude,
                                  double maximumLinearMagnitude)
   {
      getAngularAndLinearPartsFromSpatialVector(angularPart, linearPart, spatialVectorToClip);

      // Clip the angular part of the spatialVectorToClip
      subspaceSpatialVector.reshape(selectionMatrix.getNumRows(), 1);
      tempSpatialVector.reshape(SpatialVector.SIZE, 1);
      angularPart.get(0, tempSpatialVector);
      CommonOps_DDRM.mult(selectionMatrix, tempSpatialVector, subspaceSpatialVector);

      double angularPartmagnitude = NormOps_DDRM.normP2(subspaceSpatialVector);
      if (angularPartmagnitude > maximumAngularMagnitude)
         angularPart.scale(maximumAngularMagnitude / angularPartmagnitude);

      // Clip the linear part of the spatialVectorToClip
      subspaceSpatialVector.reshape(selectionMatrix.getNumRows(), 1);
      tempSpatialVector.reshape(SpatialVector.SIZE, 1);
      linearPart.get(3, tempSpatialVector);
      CommonOps_DDRM.mult(selectionMatrix, tempSpatialVector, subspaceSpatialVector);

      double linearPartMagnitude = NormOps_DDRM.normP2(subspaceSpatialVector);
      if (linearPartMagnitude > maximumAngularMagnitude)
         linearPart.scale(maximumAngularMagnitude / linearPartMagnitude);

      setSpatialVectorFromAngularAndLinearParts(spatialVectorToClip, angularPart, linearPart);
   }

   @SuppressWarnings("unused")
   private void timeDerivative(DMatrixRMaj spatialVectorRateToPack, DMatrixRMaj spatialVector, DMatrixRMaj previousSpatialVector)
   {
      CommonOps_DDRM.subtract(spatialVector, previousSpatialVector, spatialVectorRateToPack);
      CommonOps_DDRM.scale(1.0 / controlDT, spatialVectorRateToPack);
   }

   @SuppressWarnings("unused")
   private void timeIntegration(DMatrixRMaj spatialVectorToPack, DMatrixRMaj previousSpatialVector, DMatrixRMaj spatialVectorRate)
   {
      CommonOps_DDRM.add(previousSpatialVector, controlDT, spatialVectorRate, spatialVectorToPack);
   }

   private void getAngularAndLinearPartsFromSpatialVector(YoFrameVector3D angularPartToPack, YoFrameVector3D linearPartToPack, DMatrixRMaj spatialVector)
   {
      getAngularAndLinearPartsFromSpatialVector(angularPart, linearPart, spatialVector);
      angularPartToPack.setMatchingFrame(angularPart);
      linearPartToPack.setMatchingFrame(linearPart);
   }

   private void getAngularAndLinearPartsFromSpatialVector(FrameVector3D angularPartToPack, FrameVector3D linearPartToPack, DMatrixRMaj spatialVector)
   {
      angularPartToPack.setIncludingFrame(localControlFrame, 0, spatialVector);
      linearPartToPack.setIncludingFrame(localControlFrame, 3, spatialVector);
   }

   private void setSpatialVectorFromAngularAndLinearParts(DMatrixRMaj spatialVectorToPack, YoFrameVector3D yoAngularPart, YoFrameVector3D yoLinearPart)
   {
      angularPart.setIncludingFrame(yoAngularPart);
      linearPart.setIncludingFrame(yoLinearPart);
      angularPart.get(0, spatialVectorToPack);
      linearPart.get(3, spatialVectorToPack);
   }

   private void setSpatialVectorFromAngularAndLinearParts(DMatrixRMaj spatialVectorToPack, FrameVector3D angularPart, FrameVector3D linearPart)
   {
      angularPart.get(0, spatialVectorToPack);
      linearPart.get(3, spatialVectorToPack);
   }

   private void computePrivilegedJointVelocitiesForPriviligedJointAngles(DMatrixRMaj privilegedJointVelocitiesToPack, double weight)
   {
      for (int i = 0; i < numberOfDoF; i++)
      {
         yoPrivilegedJointPositionsFiltered[i].update();
         privilegedJointVelocitiesToPack.set(i,
                                             0,
                                             -2.0 * weight * (localJoints[i].getQ() - yoPrivilegedJointPositionsFiltered[i].getDoubleValue())
                                                   / jointSquaredRangeOfMotions.get(i, 0));
      }
   }

   /**
    * Compute the gradient of q times the p-norm |q - q_midRange|_p
    * 
    * @param privilegedJointVelocitiesToPack
    * @param weight
    */
   private void computePrivilegedVelocitiesForStayingAwayFromJointLimits(DMatrixRMaj privilegedJointVelocitiesToPack, double weight)
   {
      int p = exponentForPNorm.getIntegerValue();

      double sumOfPows = 0.0;
      double pThRootOfSumOfPows = 0.0;

      for (int i = 0; i < numberOfDoF; i++)
      {
         sumOfPows += MathTools.pow(Math.abs(localJoints[i].getQ() - jointAnglesAtMidRangeOfMotion.get(i, 0)), p);
      }

      pThRootOfSumOfPows = Math.pow(sumOfPows, 1.0 / ((double) p));

      for (int i = 0; i < numberOfDoF; i++)
      {
         double numerator = MathTools.pow(Math.abs(localJoints[i].getQ() - jointAnglesAtMidRangeOfMotion.get(i, 0)), p - 1) * pThRootOfSumOfPows;
         double qDotPrivileged = -weight * numerator / sumOfPows;
         privilegedJointVelocitiesToPack.set(i, 0, qDotPrivileged);
      }
   }

   private void updateFrames()
   {
      localJoints[0].getPredecessor().updateFramesRecursively();
   }

   public ReferenceFrame getControlFrame()
   {
      return originalControlFrame;
   }

   public DMatrixRMaj getSubspaceSpatialError()
   {
      return subspaceSpatialError;
   }

   public double getSpatialErrorScalar()
   {
      return NormOps_DDRM.normP2(subspaceSpatialError);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempPositionError = new FrameVector3D();
   private final DMatrixRMaj tempSpatialError = new DMatrixRMaj(SpatialVector.SIZE, 1);
   private final DMatrixRMaj tempSubspaceError = new DMatrixRMaj(SpatialVector.SIZE, 1);

   public double getNormPositionError(FramePoint3D desiredPosition)
   {
      tempPoint.setIncludingFrame(desiredPosition);
      tempPoint.changeFrame(localControlFrame);
      tempPositionError.setIncludingFrame(tempPoint);

      DMatrixRMaj selectionMatrix = inverseJacobianSolver.getSelectionMatrix();
      tempSpatialError.reshape(SpatialVector.SIZE, 1);
      tempSubspaceError.reshape(selectionMatrix.getNumRows(), 1);

      tempPositionError.get(3, tempSpatialError);
      CommonOps_DDRM.mult(selectionMatrix, tempSpatialError, tempSubspaceError);

      return NormOps_DDRM.normP2(tempSubspaceError);
   }

   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public double getNormRotationError(FrameQuaternion desiredOrientation)
   {
      tempOrientation.setIncludingFrame(desiredOrientation);
      tempOrientation.changeFrame(localControlFrame);
      errorAxisAngle.set(tempOrientation);
      errorRotationVector.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotationVector.scale(errorAxisAngle.getAngle());

      DMatrixRMaj selectionMatrix = inverseJacobianSolver.getSelectionMatrix();
      tempSpatialError.reshape(SpatialVector.SIZE, 1);
      tempSubspaceError.reshape(selectionMatrix.getNumRows(), 1);

      errorRotationVector.get(tempSpatialError);
      CommonOps_DDRM.mult(selectionMatrix, tempSpatialError, tempSubspaceError);

      return NormOps_DDRM.normP2(tempSubspaceError);
   }

   public DMatrixRMaj getDesiredJointAngles()
   {
      return desiredJointAngles;
   }

   public DMatrixRMaj getDesiredJointVelocities()
   {
      return desiredJointVelocities;
   }

   public DMatrixRMaj getDesiredJointAccelerations()
   {
      return desiredJointAccelerations;
   }

   public void getDesiredJointAngles(DMatrixRMaj desiredJointAnglesToPack)
   {
      desiredJointAnglesToPack.reshape(numberOfDoF, 1);
      desiredJointAnglesToPack.set(desiredJointAngles);
   }

   public void getDesiredJointVelocities(DMatrixRMaj desiredJointVelocitiesToPack)
   {
      desiredJointVelocitiesToPack.reshape(numberOfDoF, 1);
      desiredJointVelocitiesToPack.set(desiredJointVelocities);
   }

   public void getDesiredJointAccelerationsIntoOneDoFJoints(OneDoFJointBasics[] joints)
   {
      MultiBodySystemTools.insertJointsState(joints, JointStateType.ACCELERATION, desiredJointAccelerations);
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

   public void getDesiredEndEffectorPoseFromQDesireds(FramePose3D desiredPose, ReferenceFrame desiredFrame)
   {
      desiredPose.setToZero(localControlFrame);
      desiredPose.changeFrame(desiredFrame);
   }

   public FrameVector3D getInitialHandPoseVelocity(ReferenceFrame referenceFrame)
   {
      return new FrameVector3D(referenceFrame);
   }

   public FrameVector3D getInitialHandPoseAngularVelocity(ReferenceFrame referenceFrame)
   {
      return new FrameVector3D(referenceFrame);
   }
}
