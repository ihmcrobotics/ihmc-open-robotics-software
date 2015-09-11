package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.InverseJacobianSolver;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class TrajectoryBasedNumericalInverseKinematicsCalculator
{

   // Persistent variables
   private final double dt;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBody base;
   private final GeometricJacobian ikJacobian;
   private final TwistCalculator twistCalculator;
   private final FramePoint endEffectorPositionInFrameToControlPoseOf;
   private final FrameOrientation endEffectorOrientationInFrameToControlPoseOf;

   // TODO: YoVariableize desired variables
   private final FramePose desiredPose = new FramePose(worldFrame);
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private ReferenceFrame frameToControlPoseOf;
   private PositionTrajectoryGenerator positionTrajectoryGenerator;
   private OrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final ReferenceFrame baseFrame;
   private final ReferenceFrame baseFrameForIK;
   private final ReferenceFrame endEffectorFrame;
   private final ReferenceFrame frameToControlPoseOfForIk;

   private final RevoluteJoint[] revoluteJoints;
   private final RevoluteJoint[] revoluteJointsCopyForIK;

   private final DoubleYoVariable ikPositionErrorGain = new DoubleYoVariable("ikPositionErrorGain", registry);
   private final DoubleYoVariable ikOrientationErrorGain = new DoubleYoVariable("ikOrientationErrorGain", registry);
   private final DoubleYoVariable ikAlpha = new DoubleYoVariable("ikAlpha", registry);
   private final DoubleYoVariable maximumIKError = new DoubleYoVariable("maximumIKError", registry);
   private final DoubleYoVariable maximumAngleOutsideLimits = new DoubleYoVariable("maximumAngleOutsideLimits", registry);

   private ReferenceFrame trajectoryFrame;

   // External variables
   private final DenseMatrix64F desiredAngles;
   private final DenseMatrix64F desiredAngularVelocities;

   // Temporary variables
   private final DenseMatrix64F inverseKinematicsStep;
   private final DenseMatrix64F twistMatrix = new DenseMatrix64F(6, 1);

   private final FramePoint currentPosition = new FramePoint();
   private final FrameOrientation currentOrientation = new FrameOrientation();

   private final Vector3d linearError = new Vector3d();
   private final Vector3d angularError = new Vector3d();

   private final Point3d currentPoint = new Point3d();
   private final Matrix3d currentOrientationMatrix = new Matrix3d();
   private final Matrix3d desiredOrientationMatrix = new Matrix3d();

   private final Twist baseTwist = new Twist();
   private final Twist desiredTwist;

   private final Vector3d currentOrientationColumn = new Vector3d();
   private final Vector3d desiredOrientationColumn = new Vector3d();
   private final Vector3d columnCrossProduct = new Vector3d();
   private final Point3d desiredPositionPoint = new Point3d();

   private final Vector3d desiredVelocityVector = new Vector3d();
   private final Vector3d desiredAngularVelocityVector = new Vector3d();

   // Visualization
   private final YoFramePose yoDesiredTrajectoryPose = new YoFramePose("desiredTrajectoryPose", "", worldFrame, registry);

   private final InverseJacobianSolver inverseJacobianSolver;
   private final double lambdaLeastSquares = 0.0009;

   public TrajectoryBasedNumericalInverseKinematicsCalculator(String namePrefix, RigidBody base, RigidBody endEffector, double controlDT,
         TwistCalculator twistCalculator, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.base = base;
      this.dt = controlDT;
      this.twistCalculator = twistCalculator;
      InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.createJointPath(base, endEffector);
      revoluteJoints = ScrewTools.filterJoints(inverseDynamicsJoints, RevoluteJoint.class);
      revoluteJointsCopyForIK = ScrewTools.filterJoints(ScrewTools.cloneJointPath(inverseDynamicsJoints), RevoluteJoint.class);

      baseFrame = base.getBodyFixedFrame();
      endEffectorFrame = endEffector.getBodyFixedFrame();
      RigidBody endEffectorForIK = revoluteJointsCopyForIK[revoluteJointsCopyForIK.length - 1].getSuccessor();
      baseFrameForIK = endEffectorForIK.getBodyFixedFrame(); // TODO: Check if correct
      ReferenceFrame endEffectorFrameForIK = endEffectorForIK.getBodyFixedFrame();
      frameToControlPoseOfForIk = new ReferenceFrame(base.getName() + "ControlFrame", endEffectorFrameForIK)
      {
         private static final long serialVersionUID = -2964609854840695124L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            frameToControlPoseOf.getTransformToDesiredFrame(transformToParent, endEffectorFrame);
         }
      };

      ikJacobian = new GeometricJacobian(revoluteJointsCopyForIK[0].getPredecessor(), endEffectorForIK, baseFrameForIK);
      inverseJacobianSolver = InverseJacobianSolver.createInverseJacobianSolver(SpatialMotionVector.SIZE, revoluteJoints.length, false);

      desiredAngles = new DenseMatrix64F(ikJacobian.getNumberOfColumns(), 1);
      desiredAngularVelocities = new DenseMatrix64F(ikJacobian.getNumberOfColumns(), 1);
      inverseKinematicsStep = new DenseMatrix64F(ikJacobian.getNumberOfColumns(), 1);

      endEffectorPositionInFrameToControlPoseOf = new FramePoint(frameToControlPoseOfForIk);
      endEffectorOrientationInFrameToControlPoseOf = new FrameOrientation(frameToControlPoseOfForIk);

      desiredTwist = new Twist(endEffectorFrameForIK, baseFrame, baseFrame);

      ikPositionErrorGain.set(100.0);
      ikOrientationErrorGain.set(100.0);
      ikAlpha.set(0.05);
      maximumIKError.set(0.06);
      maximumAngleOutsideLimits.set(0.1);

      if (yoGraphicsListRegistry != null)
      {
         yoGraphicsListRegistry.registerYoGraphic("DesiredTrajectoryPose", new YoGraphicCoordinateSystem(endEffector.getName() + "DesiredTrajectoryPose", yoDesiredTrajectoryPose, 0.2));
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void initialize()
   {
      for (int i = 0; i < revoluteJointsCopyForIK.length; i++)
      {
         RevoluteJoint measured = revoluteJoints[i];
         RevoluteJoint copy = revoluteJointsCopyForIK[i];

         copy.setJointPositionVelocityAndAcceleration(measured);
         copy.updateMotionSubspace();

         desiredAngles.set(i, 0, measured.getQ());
      }

      revoluteJointsCopyForIK[0].getPredecessor().updateFramesRecursively();
      ikJacobian.compute();
      updateTrajectories(0.0);
   }

   public boolean compute(double time)
   {
      boolean inverseKinematicsIsValid = updateError();

      updateTrajectories(time);

      desiredVelocity.get(desiredVelocityVector);
      desiredAngularVelocity.get(desiredAngularVelocityVector);

      linearError.scale(ikPositionErrorGain.getDoubleValue());
      angularError.scale(ikOrientationErrorGain.getDoubleValue());
      desiredVelocityVector.add(linearError);
      desiredAngularVelocityVector.add(angularError);

      desiredTwist.set(frameToControlPoseOf, trajectoryFrame, baseFrameForIK, desiredVelocityVector, desiredAngularVelocityVector);

      transformTwistToEndEffectorFrame();

      // Calculate desired joint velocities
      desiredTwist.packMatrix(twistMatrix, 0);
      inverseJacobianSolver.solveUsingDampedLeastSquares(twistMatrix, ikJacobian.getJacobianMatrix(), lambdaLeastSquares);
      inverseKinematicsStep.set(inverseJacobianSolver.getJointspaceVelocity());

      CommonOps.addEquals(desiredAngles, dt, inverseKinematicsStep);

      for (int i = 0; i < revoluteJointsCopyForIK.length; i++)
      {
         double q = desiredAngles.get(i, 0);
         if (q < (revoluteJoints[i].getJointLimitLower() - maximumAngleOutsideLimits.getDoubleValue())
               || q > (revoluteJoints[i].getJointLimitUpper() + maximumAngleOutsideLimits.getDoubleValue()))
         {
            inverseKinematicsIsValid = false;
         }

         revoluteJointsCopyForIK[i].setQ(q);
      }

      // Update frames
      revoluteJointsCopyForIK[0].getPredecessor().updateFramesRecursively();
      ikJacobian.compute();

      if (inverseKinematicsIsValid)
      {
         updateDesiredVelocities();
      }
      else
      {
         desiredAngularVelocities.zero();
      }

      return inverseKinematicsIsValid;
   }

   private void transformTwistToEndEffectorFrame()
   {
      //TODO: Check correctness
      //      desiredTwist.changeFrame(frameToControlPoseOf);
      desiredTwist.changeBodyFrameNoRelativeTwist(endEffectorFrame);
      //      desiredTwist.changeFrame(baseFrame);

      if (trajectoryFrame == worldFrame && twistCalculator != null)
      {
         twistCalculator.packTwistOfBody(baseTwist, base);
         baseTwist.changeFrame(desiredTwist.getExpressedInFrame());
         desiredTwist.sub(baseTwist);
      }
      else
      {
         desiredTwist.changeBaseFrameNoRelativeTwist(baseFrameForIK);
      }
   }

   private void updateDesiredVelocities()
   {
      desiredTwist.set(frameToControlPoseOf, trajectoryFrame, baseFrameForIK, desiredVelocity.getVector(), desiredAngularVelocity.getVector());
      transformTwistToEndEffectorFrame();
      desiredTwist.packMatrix(twistMatrix, 0);
      inverseJacobianSolver.solveUsingDampedLeastSquares(twistMatrix, ikJacobian.getJacobianMatrix(), lambdaLeastSquares);
      desiredAngularVelocities.set(inverseJacobianSolver.getJointspaceVelocity());
   }

   private void updateTrajectories(double time)
   {
      // Compute and pack desired hand positions and velocities
      positionTrajectoryGenerator.compute(time);
      orientationTrajectoryGenerator.compute(time);

      positionTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
      orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      yoDesiredTrajectoryPose.setAndMatchFrame(desiredPosition, desiredOrientation);

      trajectoryFrame = desiredPosition.getReferenceFrame();

      desiredPosition.changeFrame(baseFrameForIK);
      desiredVelocity.changeFrame(baseFrameForIK);
      desiredOrientation.changeFrame(baseFrameForIK);
      desiredAngularVelocity.changeFrame(baseFrameForIK);
   }

   private boolean updateError()
   {
      frameToControlPoseOfForIk.update();

      currentPosition.setIncludingFrame(endEffectorPositionInFrameToControlPoseOf);
      currentPosition.changeFrame(baseFrameForIK);

      currentPosition.get(currentPoint);
      desiredPosition.get(desiredPositionPoint);

      linearError.sub(desiredPositionPoint, currentPoint);

      currentOrientation.setIncludingFrame(endEffectorOrientationInFrameToControlPoseOf);
      currentOrientation.changeFrame(baseFrameForIK);

      currentOrientation.getMatrix3d(currentOrientationMatrix);
      desiredOrientation.getMatrix3d(desiredOrientationMatrix);

      angularError.set(0.0, 0.0, 0.0);
      for (int i = 0; i < 3; i++)
      {
         currentOrientationMatrix.getColumn(i, currentOrientationColumn);
         desiredOrientationMatrix.getColumn(i, desiredOrientationColumn);
         columnCrossProduct.cross(currentOrientationColumn, desiredOrientationColumn);

         angularError.add(columnCrossProduct);
      }
      angularError.scale(0.5);

      return linearError.length() <= maximumIKError.getDoubleValue() && angularError.length() <= maximumIKError.getDoubleValue();
   }

   public RevoluteJoint[] getRevoluteJointsInOrder()
   {
      return revoluteJoints;
   }

   public void setTrajectory(PositionTrajectoryGenerator positionTrajectoryGenerator, OrientationTrajectoryGenerator orientationTrajectoryGenerator,
         ReferenceFrame frameToControlPoseOf)
   {
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.orientationTrajectoryGenerator = orientationTrajectoryGenerator;
      this.frameToControlPoseOf = frameToControlPoseOf;
   }

   public DenseMatrix64F getDesiredJointAngles()
   {
      return desiredAngles;
   }

   public DenseMatrix64F getDesiredJointVelocities()
   {
      return desiredAngularVelocities;
   }

   public FramePose getDesiredPose()
   {
      yoDesiredTrajectoryPose.getFramePoseIncludingFrame(desiredPose);
      return desiredPose;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return trajectoryFrame;
   }
}
