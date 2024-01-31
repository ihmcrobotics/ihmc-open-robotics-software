package us.ihmc.avatar.reachabilityMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
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
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.collision.CollisionTools;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReachabilityMapSolver
{
   private static final int DEFAULT_MAX_NUMBER_OF_ITERATIONS = 100;
   private static final double DEFAULT_QUALITY_THRESHOLD = 0.001;
   private static final double DEFAULT_STABILITY_THRESHOLD = 0.00002;
   private static final double DEFAULT_MIN_PROGRESSION = 0.0005;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoDouble solutionQualityThreshold = new YoDouble("solutionQualityThreshold", registry);
   private final YoDouble solutionStabilityThreshold = new YoDouble("solutionStabilityThreshold", registry);
   private final YoDouble solutionMinimumProgression = new YoDouble("solutionProgressionThreshold", registry);

   private final int numberOfTrials = 10;
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

      maximumNumberOfIterations.set(DEFAULT_MAX_NUMBER_OF_ITERATIONS);
      solutionQualityThreshold.set(DEFAULT_QUALITY_THRESHOLD);
      solutionStabilityThreshold.set(DEFAULT_STABILITY_THRESHOLD);
      solutionMinimumProgression.set(DEFAULT_MIN_PROGRESSION);

      parentRegistry.addChild(registry);
   }

   public void setCollisionModel(RobotDefinition robotDefinition)
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
      message.setEndEffectorHashCode(endEffector.hashCode());
      pose.get(message.getDesiredPositionInWorld(), message.getDesiredOrientationInWorld());

      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
      message.getControlFramePositionInEndEffector().set(controlFramePoseInEndEffector.getPosition());
      message.getControlFrameOrientationInEndEffector().set(controlFramePoseInEndEffector.getOrientation());
      if (solverForRay)
         MessageTools.packSelectionMatrix3DMessage(rayAngularSelection, message.getAngularSelectionMatrix());
      else
         MessageTools.packSelectionMatrix3DMessage(true, message.getAngularSelectionMatrix());
      commandInputManager.submitMessage(message);

      return solveAndRetry(maximumNumberOfIterations.getIntegerValue());
   }

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

      return solveAndRetry(50);
   }

   private boolean solveAndRetry(int maximumNumberOfIterations)
   {
      numberOfIterations.set(0);
      boolean success = false;
      while (!success && numberOfIterations.getValue() < numberOfTrials)
      {
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, robotArmJoints);
         success = solveOnce(maximumNumberOfIterations);
         numberOfIterations.increment();
      }
      return success;
   }

   private boolean solveOnce(int maximumNumberOfIterations)
   {
      boolean isSolutionGood = false;
      boolean isSolverStuck = false;
      double solutionQuality = Double.NaN;
      double solutionQualityLast = Double.NaN;
      double solutionQualityBeforeLast = Double.NaN;
      int iteration = 0;

      kinematicsToolboxController.updateRobotConfigurationData(defaultArmConfiguration);

      while (!isSolutionGood && iteration < maximumNumberOfIterations)
      {
         kinematicsToolboxController.update();

         KinematicsToolboxOutputStatus solution = kinematicsToolboxController.getSolution();
         solutionQuality = solution.getSolutionQuality();

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

      return isSolutionGood;
   }

   public OneDoFJointBasics[] getRobotArmJoints()
   {
      return robotArmJoints;
   }

   public FramePose3D getControlFramePoseInEndEffector()
   {
      return controlFramePoseInEndEffector;
   }
}
