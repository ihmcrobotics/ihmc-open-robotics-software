package us.ihmc.avatar.reachabilityMap;

import java.util.Collections;
import java.util.Random;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReachabilityMapSolver
{
   private static final int DEFAULT_MAX_NUMBER_OF_ITERATIONS = 100;
   private static final double DEFAULT_QUALITY_THRESHOLD = 0.001;
   private static final double DEFAULT_STABILITY_THRESHOLD = 0.00002;
   private static final double DEFAULT_MIN_PROGRESSION = 0.0005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);
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
   private final RigidBodyTransform controlFramePoseInEndEffector = new RigidBodyTransform();
   private final SelectionMatrix3D angularSelection = new SelectionMatrix3D(null, true, true, true);
   private final RobotConfigurationData defaultArmConfiguration;

   public ReachabilityMapSolver(OneDoFJointBasics[] robotArmJoints, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
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

   public void setControlFramePose(RigidBodyTransform controlFramePose)
   {
      controlFramePoseInEndEffector.set(controlFramePose);
   }

   public void setAngularSelection(boolean selectX, boolean selectY, boolean selectZ)
   {
      angularSelection.setAxisSelection(selectX, selectY, selectZ);
   }

   public boolean solveFor(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      kinematicsToolboxController.requestInitialize();
      FramePoint3D desiredPosition = new FramePoint3D(position);
      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion desiredOrientation = new FrameQuaternion(orientation);
      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(endEffector, desiredPosition, desiredOrientation);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1.0));
      message.getControlFramePositionInEndEffector().set(controlFramePoseInEndEffector.getTranslationVector());
      message.getControlFrameOrientationInEndEffector().set(controlFramePoseInEndEffector.getRotationMatrix());
      message.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(angularSelection));
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
      message.getControlFramePositionInEndEffector().set(controlFramePoseInEndEffector.getTranslationVector());
      message.getControlFrameOrientationInEndEffector().set(controlFramePoseInEndEffector.getRotationMatrix());
      commandInputManager.submitMessage(message);

      return solveAndRetry(50);
   }

   private boolean solveAndRetry(int maximumNumberOfIterations)
   {
      int tryNumber = 0;
      boolean success = false;
      while (!success && tryNumber++ < numberOfTrials)
      {
         MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, robotArmJoints);
         success = solveOnce(maximumNumberOfIterations);
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

      if (isSolutionGood)
      {
         for (int i = 0; i < robotArmJoints.length; i++)
            robotArmJoints[i].setQ(kinematicsToolboxController.getDesiredOneDoFJoint()[i].getQ());
      }

      return isSolutionGood;
   }

   public OneDoFJointBasics[] getRobotArmJoints()
   {
      return robotArmJoints;
   }
}
