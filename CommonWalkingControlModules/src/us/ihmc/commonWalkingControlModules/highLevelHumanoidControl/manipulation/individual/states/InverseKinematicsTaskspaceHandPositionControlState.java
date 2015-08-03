package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.HashMap;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TrajectoryBasedNumericalInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.filters.RateLimitedYoVariable;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;

public class InverseKinematicsTaskspaceHandPositionControlState extends TrajectoryBasedTaskspaceHandControlState
{
   private final boolean lowLevelVersion;

   private final ControlStatusProducer controlStatusProducer;
   private final RobotSide robotSide;
   private final TrajectoryBasedNumericalInverseKinematicsCalculator inverseKinematicsCalculator;

   private final RevoluteJoint[] oneDoFJoints;
   private final BooleanYoVariable inverseKinematicsSolutionIsValid;

   private final boolean[] doIntegrateDesiredAccerations;
   private final double controlDT;
   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
   private final HashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations = new HashMap<OneDoFJoint, RateLimitedYoVariable>();
   private final DoubleYoVariable maxAccelerationArmJointspace;

   private final DoubleYoVariable doneTrajectoryTime;
   private final DoubleYoVariable holdPositionDuration;

   private ReferenceFrame trackingFrame;
   private PoseTrajectoryGenerator poseTrajectoryGenerator;

   public InverseKinematicsTaskspaceHandPositionControlState(String namePrefix, HandControlState stateEnum, RobotSide robotSide,
         MomentumBasedController momentumBasedController, int jacobianId, RigidBody base, RigidBody endEffector, YoGraphicsListRegistry yoGraphicsListRegistry,
         ArmControllerParameters armControllerParameters, ControlStatusProducer controlStatusProducer, YoPIDGains gains, double controlDT,
         YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, parentRegistry);
      this.controlStatusProducer = controlStatusProducer;
      this.robotSide = robotSide;
      inverseKinematicsCalculator = new TrajectoryBasedNumericalInverseKinematicsCalculator(namePrefix, base, endEffector, controlDT, momentumBasedController.getTwistCalculator(),
            parentRegistry, yoGraphicsListRegistry);

      inverseKinematicsSolutionIsValid = new BooleanYoVariable("inverseKinematicsSolutionIsValid", registry);

      oneDoFJoints = inverseKinematicsCalculator.getRevoluteJointsInOrder();

      lowLevelVersion = armControllerParameters.doLowLevelPositionControl();

      doIntegrateDesiredAccerations = new boolean[oneDoFJoints.length];

      for (OneDoFJoint joint : oneDoFJoints)
      {
         String suffix = joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression();
         PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(), gains.getYoMaxIntegralError(), suffix, registry);
         pidControllers.put(joint, pidController);

         RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(joint.getName() + "Acceleration" + robotSide, registry,
               gains.getYoMaximumJerk(), controlDT);
         rateLimitedAccelerations.put(joint, rateLimitedAcceleration);
      }

      this.controlDT = controlDT;
      maxAccelerationArmJointspace = gains.getYoMaximumAcceleration();


      doneTrajectoryTime = new DoubleYoVariable(namePrefix + "DoneTrajectoryTime", registry);
      holdPositionDuration = new DoubleYoVariable(namePrefix + "HoldPositionDuration", registry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      poseTrajectoryGenerator.showVisualization();
      poseTrajectoryGenerator.initialize();
      doneTrajectoryTime.set(Double.NaN);

      inverseKinematicsCalculator.setTrajectory(poseTrajectoryGenerator, poseTrajectoryGenerator, trackingFrame);
      inverseKinematicsCalculator.initialize();
      inverseKinematicsSolutionIsValid.set(true);
      if (lowLevelVersion)
      {
         enableLowLevelPositionControl();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      if (lowLevelVersion)
      {
         disableLowLevelPositionControl();
      }
   }

   private void enableLowLevelPositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         doIntegrateDesiredAccerations[i] = joint.getIntegrateDesiredAccelerations();
         joint.setIntegrateDesiredAccelerations(false);
         joint.setUnderPositionControl(true);

      }
   }

   private void disableLowLevelPositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccerations[i]);
         joint.setUnderPositionControl(false);
      }
   }

   @Override
   public void doAction()
   {
      if (poseTrajectoryGenerator.isDone())
         recordDoneTrajectoryTime();

      if (inverseKinematicsSolutionIsValid.getBooleanValue())
      {
         inverseKinematicsSolutionIsValid.set(inverseKinematicsCalculator.compute(getTimeInCurrentState()));

         if (!inverseKinematicsSolutionIsValid.getBooleanValue())
         {
            controlStatusProducer.notifyHandTrajectoryInfeasible(robotSide);
         }
      }

      RevoluteJoint[] revoluteJoints = inverseKinematicsCalculator.getRevoluteJointsInOrder();
      DenseMatrix64F desiredJointAngles = inverseKinematicsCalculator.getDesiredJointAngles();
      DenseMatrix64F desiredJointVelocities = inverseKinematicsCalculator.getDesiredJointVelocities();

      for (int i = 0; i < revoluteJoints.length; i++)
      {
         OneDoFJoint joint = revoluteJoints[i];
         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         double desiredPosition = desiredJointAngles.get(i, 0);
         double desiredVelocity = desiredJointVelocities.get(i, 0);

         double desiredAcceleration = 0;
         if (lowLevelVersion)
         {
            joint.setqDesired(desiredPosition);
            joint.setQdDesired(desiredVelocity);
         }

         PIDController pidController = pidControllers.get(joint);
         desiredAcceleration = pidController.computeForAngles(currentPosition, desiredPosition, currentVelocity, desiredVelocity, controlDT);
         desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAccelerationArmJointspace.getDoubleValue());

         RateLimitedYoVariable rateLimitedAcceleration = rateLimitedAccelerations.get(joint);
         rateLimitedAcceleration.update(desiredAcceleration);
         desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

         momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
      }
   }

   private void recordDoneTrajectoryTime()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
      {
         doneTrajectoryTime.set(getTimeInCurrentState());
      }
   }

   @Override
   public boolean isDone()
   {
      if (Double.isNaN(doneTrajectoryTime.getDoubleValue()))
         return false;
      
      boolean isTrajectoryDone =  getTimeInCurrentState() > doneTrajectoryTime.getDoubleValue() + holdPositionDuration.getDoubleValue();
   
      return isTrajectoryDone || !inverseKinematicsSolutionIsValid.getBooleanValue();
   }

   @Override
   public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator)
   {
      this.poseTrajectoryGenerator = poseTrajectoryGenerator;
   }

   @Override
   public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule)
   {
      trackingFrame = handRigidBodySpatialAccelerationControlModule.getTrackingFrame();
   }

   @Override
   public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
   {
   }

   @Override
   public void setHoldPositionDuration(double holdPositionDuration)
   {
     this.holdPositionDuration.set(holdPositionDuration);
   }

   @Override
   public FramePose getDesiredPose()
   {
      return inverseKinematicsCalculator.getDesiredPose();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return inverseKinematicsCalculator.getReferenceFrame();
   }
}
