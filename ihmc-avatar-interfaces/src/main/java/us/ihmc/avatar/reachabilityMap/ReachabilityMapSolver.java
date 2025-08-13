package us.ihmc.avatar.reachabilityMap;

import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.IKRobotStateUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand.ActivationState;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollisionTools;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Arrays;
import java.util.Collections;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public class ReachabilityMapSolver
{
   private static final int DEFAULT_MAX_TRIALS = 10;
   private static final int DEFAULT_POSITION_MAX_ITERATIONS = 50;
   private static final int DEFAULT_MAX_NUMBER_OF_ITERATIONS = 100;
   private static final int DEFAULT_MAX_NUMBER_OF_TOTAL_ITERATIONS = 250;
   private static final double DEFAULT_QUALITY_THRESHOLD = 0.001;
   private static final double DEFAULT_STABILITY_THRESHOLD = 0.00002;
   private static final double DEFAULT_MIN_PROGRESSION = 0.0005;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   /** This is the number of iterations the solver will run to attempt to reach the desired goal */
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);
   private final YoInteger maximumNumberOfTotalIterations = new YoInteger("maximumNumberOfTotalIterations", registry);
   /** This is the number of trials the solver will run to attempt to reach the desired goal, where each trial starts from a random configuration and then
    *  iterates*/
   private final YoInteger maximumNumberOfTrials = new YoInteger("maximumNumberOfTrials", registry);
   private final YoInteger numberOfTrials = new YoInteger("numberOfTrials", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoDouble solutionQualityThreshold = new YoDouble("solutionQualityThreshold", registry);
   private final YoDouble solutionStabilityThreshold = new YoDouble("solutionStabilityThreshold", registry);
   private final YoDouble solutionMinimumProgression = new YoDouble("solutionProgressionThreshold", registry);

   private final Random random = new Random(645216L);

   private final KinematicsToolboxController kinematicsToolboxController;
   private final CommandInputManager commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
   private final StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
   private final RigidBodyBasics endEffector;
   private final OneDoFJointBasics[] robotArmJoints;
   private final FramePose3D controlFramePoseInEndEffector = new FramePose3D();
   private final SelectionMatrix3D rayAngularSelection = new SelectionMatrix3D(null, false, true, true); // Assume by default that X is orthogonal to the palm
   private final RobotConfigurationData defaultArmConfiguration;
   private final String cloneSuffix;

   private Predicate<OneDoFJointReadOnly[]> solutionValidityChecker = null;

   public ReachabilityMapSolver(String cloneSuffix,
                                OneDoFJointBasics[] robotArmJoints,
                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.cloneSuffix = cloneSuffix;
      this.robotArmJoints = robotArmJoints;
      endEffector = robotArmJoints[robotArmJoints.length - 1].getSuccessor();
      kinematicsToolboxController = new KinematicsToolboxController(commandInputManager,
                                                                    statusOutputManager,
                                                                    null,
                                                                    robotArmJoints,
                                                                    Collections.singleton(endEffector),
                                                                    1.0e-3,
                                                                    yoGraphicsListRegistry,
                                                                    registry);
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(MultiBodySystemTools.getRootBody(endEffector)));

      defaultArmConfiguration = RobotConfigurationDataFactory.create(robotArmJoints, new ForceSensorDefinition[0], new IMUDefinition[0]);
      RobotConfigurationDataFactory.packJointState(defaultArmConfiguration, robotArmJoints);
      kinematicsToolboxController.setDesiredRobotStateUpdater(IKRobotStateUpdater.wrap(defaultArmConfiguration));

      maximumNumberOfIterations.set(DEFAULT_MAX_NUMBER_OF_ITERATIONS);
      maximumNumberOfTotalIterations.set(DEFAULT_MAX_NUMBER_OF_TOTAL_ITERATIONS);
      maximumNumberOfTrials.set(DEFAULT_MAX_TRIALS);
      solutionQualityThreshold.set(DEFAULT_QUALITY_THRESHOLD);
      solutionStabilityThreshold.set(DEFAULT_STABILITY_THRESHOLD);
      solutionMinimumProgression.set(DEFAULT_MIN_PROGRESSION);

      parentRegistry.addChild(registry);
   }

   /**
    * Deprecated. Please use {@link #addCollisionModel(RobotDefinition)}.
    */
   @Deprecated
   public void setCollisionModel(RobotDefinition robotDefinition)
   {
      addCollisionModel(robotDefinition);
   }

   public void addCollisionModel(RobotDefinition robotDefinition)
   {
      Set<RigidBodyBasics> solverRigidBodies = Arrays.stream(robotArmJoints).map(JointBasics::getSuccessor).collect(Collectors.toSet());
      //    solverRigidBodies.add(robotArmJoints[0].getPredecessor());

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(endEffector);

      for (RigidBodyDefinition rigidBodyDef : robotDefinition.getAllRigidBodies())
      {
         if (rigidBodyDef.getCollisionShapeDefinitions() == null)
            continue;

         RigidBodyBasics rigidBody = MultiBodySystemTools.findRigidBody(rootBody, rigidBodyDef.getName() + cloneSuffix);
         boolean staticCollidable = !solverRigidBodies.contains(rigidBody);
         ReferenceFrame shapeFrame = rigidBody.isRootBody() ? rigidBody.getBodyFixedFrame() : rigidBody.getParentJoint().getFrameAfterJoint();

         for (CollisionShapeDefinition collisionDef : rigidBodyDef.getCollisionShapeDefinitions())
         {
            FrameShape3DReadOnly shape = CollisionTools.toFrameShape3D(collisionDef.getOriginPose(), shapeFrame, collisionDef.getGeometryDefinition());
            long collisionMask = collisionDef.getCollisionMask();
            long collisionGroup = collisionDef.getCollisionGroup();

            if (staticCollidable)
               kinematicsToolboxController.registerStaticCollidable(new Collidable(null, collisionMask, collisionGroup, shape));
            else
               kinematicsToolboxController.registerRobotCollidable(new Collidable(rigidBody, collisionMask, collisionGroup, shape));
         }
      }
   }

   public void setControlFramePoseInParentJoint(Pose3DReadOnly controlFramePose)
   {
      controlFramePoseInEndEffector.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), controlFramePose);
      controlFramePoseInEndEffector.changeFrame(endEffector.getBodyFixedFrame());
   }

   public void setControlFramePoseInParentJoint(RigidBodyTransformReadOnly controlFramePose)
   {
      controlFramePoseInEndEffector.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), controlFramePose);
      controlFramePoseInEndEffector.changeFrame(endEffector.getBodyFixedFrame());
   }

   public void setRayAxis(Axis3D rayAxis)
   {
      setRaySolveAngularSelection(rayAxis != Axis3D.X, rayAxis != Axis3D.Y, rayAxis != Axis3D.Z);
   }

   public void setRaySolveAngularSelection(boolean selectX, boolean selectY, boolean selectZ)
   {
      rayAngularSelection.setAxisSelection(selectX, selectY, selectZ);
   }

   public void enableJointTorqueAnalysis(boolean considerJointTorqueLimits)
   {
      kinematicsToolboxController.getActiveOptimizationSettings().setComputeJointTorques(ActivationState.ENABLED);
      kinematicsToolboxController.getActiveOptimizationSettings().setJointTorqueWeight(0.01);
      if (considerJointTorqueLimits)
      {
         addSolutionValidityChecker(joints ->
                                    {
                                       for (OneDoFJointReadOnly joint : joints)
                                       {
                                          if (joint.getTau() > joint.getEffortLimitUpper())
                                             return false;
                                          if (joint.getTau() < joint.getEffortLimitLower())
                                             return false;
                                       }
                                       return true;
                                    });
      }
   }

   public void addSolutionValidityChecker(Predicate<OneDoFJointReadOnly[]> checker)
   {
      if (solutionValidityChecker == null)
         solutionValidityChecker = checker;
      else
         solutionValidityChecker = solutionValidityChecker.and(checker);
   }

   public boolean solveForRay(FramePose3DReadOnly pose)
   {
      return solveFor(pose, true);
   }

   public boolean solveForPose(FramePose3DReadOnly pose)
   {
      return solveFor(pose, false);
   }

   private boolean solveFor(FramePose3DReadOnly pose, boolean solverForRay)
   {
      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      kinematicsToolboxController.requestInitialize();

      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      // Set the hash code of the end effector.
      message.setEndEffectorHashCode(endEffector.hashCode());
      // Set the desired pose of the end effector.
      pose.get(message.getDesiredPositionInWorld(), message.getDesiredOrientationInWorld());
      // Set the control frame pose in the end effector frame.
      controlFramePoseInEndEffector.get(message.getControlFramePositionInEndEffector(), message.getControlFrameOrientationInEndEffector());

      // Set a uniform weight for the solver.
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));

      if (solverForRay)
      {
         // If doing a ray solution, we select the y and z orientation, but not the x orientation, as well as set a null frame to the selection. The x-axis is
         // pointed along the ray, so rotation about the x-axis is left free.
         MessageTools.packSelectionMatrix3DMessage(rayAngularSelection, message.getAngularSelectionMatrix());
      }
      else
      {
         // If doing not a ray selection, we're doing a pose selection, and we care about all of the axes of the orientations.
         MessageTools.packSelectionMatrix3DMessage(true, message.getAngularSelectionMatrix());
      }
      commandInputManager.submitMessage(message);

      return solveAndRetry(maximumNumberOfIterations.getIntegerValue());
   }

   /**
    * This computes the necessary kinematic configuration of the system to reach the desired position. This configuration must respect the joint limits and
    * collisions defined in the robot definition. It will perform up to 50 iterations to find the right solution.
    *
    * <p>
    *    This desired position must be defined for the control frame in the end effector. To change this control frame, please use
    *    {@link #setControlFramePoseInParentJoint(Pose3DReadOnly)}. This can be thought of as the "palm" frame if the end effector is the hand.
    * </p>
    * <p>
    *    Any orientation is acceptable.
    * </p>
    *
    * @param position desired position of the control frame of the end effector.
    * @return true if valid solution was found, false otherwise.
    */
   public boolean solveFor(FramePoint3DReadOnly position)
   {
      kinematicsToolboxController.requestInitialize();
      FramePoint3D desiredPosition = new FramePoint3D(position);
      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(endEffector, desiredPosition);
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
      message.getControlFramePositionInEndEffector().set(controlFramePoseInEndEffector.getPosition());
      message.getControlFrameOrientationInEndEffector().set(controlFramePoseInEndEffector.getOrientation());
      commandInputManager.submitMessage(message);

      return solveAndRetry(DEFAULT_POSITION_MAX_ITERATIONS);
   }

   private boolean solveAndRetry(int maximumNumberOfIterations)
   {
      numberOfTrials.set(0);
      numberOfIterations.set(0);
      boolean success = false;
      while (!success && numberOfTrials.getValue() < maximumNumberOfTrials.getValue() && numberOfIterations.getValue() < maximumNumberOfTotalIterations.getValue())
      {
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, robotArmJoints);
         TrialResult result = solveOnce(maximumNumberOfIterations);
         success = result.isSolutionGood;
         numberOfTrials.increment();
         numberOfIterations.add(result.iterations);
      }
      return success;
   }

   private TrialResult solveOnce(int maximumNumberOfIterations)
   {
      boolean isSolutionGood = false;
      boolean isSolverStuck = false;
      double solutionQuality = Double.NaN;
      double solutionQualityLast = Double.NaN;
      double solutionQualityBeforeLast = Double.NaN;
      int iteration = 0;

      while (!isSolutionGood && iteration < maximumNumberOfIterations)
      {
         kinematicsToolboxController.update();

         KinematicsToolboxOutputStatus solution = kinematicsToolboxController.getSolution();
         solutionQuality = solution.getSolutionQuality();

         // If the quality is negative, no solution was found. Return, and start from a new random configuration.
         if (solutionQuality < 0.0)
            return new TrialResult(false, solutionQuality, iteration);

         if (!Double.isNaN(solutionQualityLast))
         {
            double deltaSolutionQualityLast = Math.abs(solutionQuality - solutionQualityLast);
            double deltaSolutionQualityBeforeLast = Math.abs(solutionQuality - solutionQualityBeforeLast);

            boolean isSolutionStable = deltaSolutionQualityLast < solutionStabilityThreshold.getDoubleValue();
            boolean isSolutionQualityHigh = solutionQuality < solutionQualityThreshold.getDoubleValue();

            isSolutionGood = isSolutionStable && isSolutionQualityHigh;

            if (!isSolutionQualityHigh)
            {
               // current solution quality should be compared with not only the last value but also the value before the last.
               boolean stuckLast = (deltaSolutionQualityLast / solutionQuality) < solutionMinimumProgression.getDoubleValue();
               boolean stuckBeforeLast = (deltaSolutionQualityBeforeLast / solutionQuality) < solutionMinimumProgression.getDoubleValue();

               isSolverStuck = stuckLast || stuckBeforeLast;
            }
            else
               isSolverStuck = false;
         }

         solutionQualityBeforeLast = solutionQualityLast;
         solutionQualityLast = solutionQuality;

         iteration++;

         if (isSolverStuck)
            break;
      }

      if (isSolutionGood && solutionValidityChecker != null)
      {
         isSolutionGood = solutionValidityChecker.test(kinematicsToolboxController.getDesiredOneDoFJoints());
      }

      if (isSolutionGood)
      {
         for (int i = 0; i < robotArmJoints.length; i++)
         {
            robotArmJoints[i].setQ(kinematicsToolboxController.getDesiredOneDoFJoints()[i].getQ());
            robotArmJoints[i].setTau(kinematicsToolboxController.getDesiredOneDoFJoints()[i].getTau());
         }
      }

      return new TrialResult(isSolutionGood, solutionQuality, iteration);
   }

   public OneDoFJointBasics[] getRobotArmJoints()
   {
      return robotArmJoints;
   }

   public FramePose3D getControlFramePoseInEndEffector()
   {
      return controlFramePoseInEndEffector;
   }

   private record TrialResult(boolean isSolutionGood, double solutionQuality, int iterations)
      {
      }
}
