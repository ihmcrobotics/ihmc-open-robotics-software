package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.HashMap;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TrajectoryBasedNumericalInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.filters.RateLimitedYoVariable;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;


public class InverseKinematicsTaskspaceHandPositionControlState extends TaskspaceHandPositionControlState
{
   private final double controlDT;
   private final ControlStatusProducer controlStatusProducer;
   private final RobotSide robotSide;
   private final TrajectoryBasedNumericalInverseKinematicsCalculator inverseKinematicsCalculator;
   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
   private final HashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations = new HashMap<OneDoFJoint, RateLimitedYoVariable>();
   private final DoubleYoVariable maxAccelerationArmJointspace;
   
   private final BooleanYoVariable inverseKinematicsSolutionIsValid;

   public InverseKinematicsTaskspaceHandPositionControlState(String namePrefix, HandControlState stateEnum, RobotSide robotSide,
         MomentumBasedController momentumBasedController, int jacobianId, RigidBody base, RigidBody endEffector,
         YoGraphicsListRegistry yoGraphicsListRegistry, ArmControllerParameters armControllerParameters,
         ControlStatusProducer controlStatusProducer, YoPIDGains gains, double controlDT, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, yoGraphicsListRegistry, parentRegistry);
      this.controlDT = controlDT;
      this.controlStatusProducer = controlStatusProducer;
      this.robotSide = robotSide;
      inverseKinematicsCalculator = new TrajectoryBasedNumericalInverseKinematicsCalculator(base, endEffector, controlDT, momentumBasedController.getTwistCalculator(), parentRegistry, yoGraphicsListRegistry);
      
      inverseKinematicsSolutionIsValid = new BooleanYoVariable("inverseKinematicsSolutionIsValid", registry);

      maxAccelerationArmJointspace = gains.getYoMaximumAcceleration();

      for (OneDoFJoint joint : inverseKinematicsCalculator.getRevoluteJointsInOrder())
      {
         String suffix = joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression();
         PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(), gains.getYoMaxIntegralError(), suffix, registry);
         pidControllers.put(joint, pidController);

         RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(joint.getName() + "Acceleration" + robotSide, registry, gains.getYoMaximumJerk(), controlDT);
         rateLimitedAccelerations.put(joint, rateLimitedAcceleration);
      }
   }

   @Override
   protected SpatialAccelerationVector computeDesiredSpatialAcceleration()
   {
      throw new RuntimeException("Not controlling task space");
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      inverseKinematicsCalculator.initialize();
      inverseKinematicsSolutionIsValid.set(true);
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public void doAction()
   {
      if(inverseKinematicsSolutionIsValid.getBooleanValue())
      {
         inverseKinematicsSolutionIsValid.set(inverseKinematicsCalculator.compute(getTimeInCurrentState()));
         if(!inverseKinematicsSolutionIsValid.getBooleanValue())
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

         PIDController pidController = pidControllers.get(joint);
         double desiredAcceleration = pidController.computeForAngles(currentPosition, desiredPosition, currentVelocity, desiredVelocity, controlDT);

         desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAccelerationArmJointspace.getDoubleValue());

         RateLimitedYoVariable rateLimitedAcceleration = rateLimitedAccelerations.get(joint);
         rateLimitedAcceleration.update(desiredAcceleration);
         desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

         momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
      }
   }

   @Override
   public boolean isDone()
   {
      return super.isDone() || !inverseKinematicsSolutionIsValid.getBooleanValue();
   }
   
   @Override
   public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator,
         RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule)
   {
      super.setTrajectory(poseTrajectoryGenerator, rigidBodySpatialAccelerationControlModule);
      inverseKinematicsCalculator.setTrajectory(poseTrajectoryGenerator, poseTrajectoryGenerator, rigidBodySpatialAccelerationControlModule.getTrackingFrame());
   }
}
