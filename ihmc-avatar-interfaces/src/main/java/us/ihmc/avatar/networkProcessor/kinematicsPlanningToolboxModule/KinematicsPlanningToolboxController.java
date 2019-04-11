package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.fest.swing.util.Pair;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsPlanningToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class KinematicsPlanningToolboxController extends ToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final boolean DEBUG = true;

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager commandInputManager;

   private final List<String> armJointNames = new ArrayList<String>();
   private final Map<String, Pair<Double, Double>> armJointVelocityLimitMap;
   private final WholeBodyTrajectoryPointCalculator fullRobotModelTrajectoryCalculator;
   private static final double searchingTimeTickForVelocityBound = 0.002;
   private static final boolean useKeyFrameTimeOptimizerIfJointVelocityExceedLimits = true;

   private final KinematicsPlanningToolboxOutputStatus solution;
   private final KinematicsPlanningToolboxOutputConverter outputConverter;

   private final YoBoolean isDone;

   private final List<KinematicsPlanningToolboxRigidBodyCommand> rigidBodyCommands;
   private final AtomicReference<KinematicsPlanningToolboxCenterOfMassCommand> centerOfMassCommand;
   private final AtomicReference<KinematicsToolboxConfigurationCommand> configurationCommand;

   private final List<RigidBodyBasics> ikRigidBodies;
   private final Map<RigidBodyBasics, List<KinematicsToolboxRigidBodyMessage>> ikRigidBodyMessageMap;
   private final List<KinematicsToolboxCenterOfMassMessage> ikCenterOfMassMessages;
   private final AtomicReference<KinematicsToolboxConfigurationMessage> ikConfigurationMessage;

   private static final double keyFrameTimeEpsilon = 0.01;
   private final TDoubleArrayList keyFrameTimes;

   private final HumanoidKinematicsToolboxController ikController;
   private final KinematicsToolboxOutputStatus initialRobotConfiguration;
   private final CommandInputManager ikCommandInputManager = new CommandInputManager(getClass().getSimpleName(), KinematicsToolboxModule.supportedCommands());

   private final YoInteger indexOfCurrentKeyFrame;
   private final YoDouble totalComputationTime;

   private final SolutionQualityConvergenceDetector solutionQualityConvergenceDetector;

   public KinematicsPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.desiredFullRobotModel = fullRobotModel;

      armJointVelocityLimitMap = new HashMap<>();
      fullRobotModelTrajectoryCalculator = new WholeBodyTrajectoryPointCalculator(drcRobotModel);

      solution = HumanoidMessageTools.createKinematicsPlanningToolboxOutputStatus();
      solution.setDestination(-1);
      outputConverter = new KinematicsPlanningToolboxOutputConverter(drcRobotModel);

      this.commandInputManager = commandInputManager;

      rigidBodyCommands = new ArrayList<KinematicsPlanningToolboxRigidBodyCommand>();
      centerOfMassCommand = new AtomicReference<>(null);
      configurationCommand = new AtomicReference<>(null);

      ikRigidBodies = new ArrayList<RigidBodyBasics>();
      ikRigidBodyMessageMap = new HashMap<>();
      ikCenterOfMassMessages = new ArrayList<KinematicsToolboxCenterOfMassMessage>();
      ikConfigurationMessage = new AtomicReference<>(null);

      keyFrameTimes = new TDoubleArrayList();

      isDone = new YoBoolean("isDone", parentRegistry);

      ikCommandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));
      ikController = new HumanoidKinematicsToolboxController(ikCommandInputManager, statusOutputManager, fullRobotModel, yoGraphicsListRegistry,
                                                             parentRegistry);
      initialRobotConfiguration = MessageTools.createKinematicsToolboxOutputStatus(ikController.getDesiredOneDoFJoint());

      indexOfCurrentKeyFrame = new YoInteger("indexOfCurrentKeyFrame", parentRegistry);
      totalComputationTime = new YoDouble("totalComputationTime", parentRegistry);

      SolutionQualityConvergenceSettings optimizationSettings = new KinematicsPlanningToolboxOptimizationSettings();
      solutionQualityConvergenceDetector = new SolutionQualityConvergenceDetector(optimizationSettings, parentRegistry);

   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      indexOfCurrentKeyFrame.set(0);
      totalComputationTime.set(0);

      rigidBodyCommands.clear();
      centerOfMassCommand.set(null);
      configurationCommand.set(null);

      if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxInputCommand.class))
      {
         KinematicsPlanningToolboxInputCommand command = commandInputManager.pollNewestCommand(KinematicsPlanningToolboxInputCommand.class);
         List<KinematicsPlanningToolboxRigidBodyCommand> rigidBodyCommands = command.getRigidBodyCommands();
         if (rigidBodyCommands.size() == 0)
         {
            return false;
         }
         else
         {
            for (int i = 0; i < rigidBodyCommands.size(); i++)
               this.rigidBodyCommands.add(rigidBodyCommands.get(i));
         }

         KinematicsPlanningToolboxCenterOfMassCommand centerOfMassCommand = command.getCenterOfMassCommand();
         this.centerOfMassCommand.set(centerOfMassCommand);

         KinematicsToolboxConfigurationCommand configurationCommand = command.getKinematicsConfigurationCommand();
         this.configurationCommand.set(configurationCommand);

      }
      else
      {
         if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxRigidBodyCommand.class))
         {
            List<KinematicsPlanningToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsPlanningToolboxRigidBodyCommand.class);
            for (int i = 0; i < commands.size(); i++)
               rigidBodyCommands.add(commands.get(i));
         }
         else
            return false;

         if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxCenterOfMassCommand.class))
         {
            KinematicsPlanningToolboxCenterOfMassCommand comCommand = commandInputManager.pollNewestCommand(KinematicsPlanningToolboxCenterOfMassCommand.class);
            centerOfMassCommand.set(comCommand);
         }

         if (commandInputManager.isNewCommandAvailable(KinematicsToolboxConfigurationCommand.class))
         {
            KinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxConfigurationCommand.class);
            configurationCommand.set(command);
         }
      }

      if (!updateInitialRobotConfigurationToIKController())
         return false;

      if (!updateToolboxConfiguration())
         return false;

      if (!updateIKMessages())
         return false;

      ikController.updateFootSupportState(true, true);
      boolean initialized = ikController.initialize();
      if (!initialized)
         throw new RuntimeException("Could not initialize the " + KinematicsToolboxController.class.getSimpleName());

      armJointVelocityLimitMap.clear();
      for (RobotSide robotSide : RobotSide.values)
      {
         JointBasics armJoint = desiredFullRobotModel.getHand(robotSide).getParentJoint();
         while (armJoint.getPredecessor() != desiredFullRobotModel.getElevator())
         {
            String armJointName = armJoint.getName();
            if (armJointName.contains(robotSide.getLowerCaseName()))
            {
               armJointNames.add(armJointName);
               OneDoFJointBasics oneDoFJointByName = desiredFullRobotModel.getOneDoFJointByName(armJointName);
               Pair<Double, Double> velocityLimit = new Pair<Double, Double>(oneDoFJointByName.getVelocityLimitLower(),
                                                                             oneDoFJointByName.getVelocityLimitUpper());
               armJointVelocityLimitMap.put(armJointName, velocityLimit);
            }
            armJoint = armJoint.getPredecessor().getParentJoint();
         }
      }

      solution.setPlanId(-1);
      solution.getKeyFrameTimes().clear();
      solution.getRobotConfigurations().clear();
      solution.setSolutionQuality(0.0);
      solution.getKeyFrameTimes().add(0.0);
      solution.getRobotConfigurations().add().set(initialRobotConfiguration);

      for (int i = 0; i < getNumberOfKeyFrames(); i++)
         solution.getKeyFrameTimes().add(keyFrameTimes.get(i));

      solutionQualityConvergenceDetector.initialize();
      submitKeyFrameMessages();

      if (DEBUG)
         System.out.println("Initializing is done");
      return true;
   }

   private boolean updateIKMessages()
   {
      int numberOfKeyFrames = keyFrameTimes.size();
      int numberOfInterpolatedPoints;
      if (numberOfKeyFrames == 1)
         numberOfInterpolatedPoints = 2;
      else if (numberOfKeyFrames == 2)
         numberOfInterpolatedPoints = 1;
      else
         return true;

      // re-create key frame times with interpolated times.
      TDoubleArrayList keyFrameTimesBuffer = new TDoubleArrayList();
      keyFrameTimesBuffer.addAll(keyFrameTimes);
      keyFrameTimes.clear();
      double t1 = 0.0;
      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         for (int j = 0; j < numberOfInterpolatedPoints; j++)
         {
            double alpha = ((double) j + 1) / (numberOfInterpolatedPoints + 1);
            double t2 = keyFrameTimesBuffer.get(i);
            double keyFrameTime = EuclidCoreTools.interpolate(t1, t2, alpha);

            keyFrameTimes.add(keyFrameTime);
         }
         t1 = keyFrameTimesBuffer.get(i);
         keyFrameTimes.add(t1);
      }

      // create interpolated way point poses for each rigid bodies.
      for (int i = 0; i < ikRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = ikRigidBodies.get(i);
         List<KinematicsToolboxRigidBodyMessage> ikRigidBodyMessages = ikRigidBodyMessageMap.get(rigidBody);
         List<Pose3D> ikRigidBodyPoses = new ArrayList<Pose3D>();
         Pose3D currentPose = new Pose3D(rigidBody.getBodyFixedFrame().getTransformToDesiredFrame(worldFrame));
         for (int j = 0; j < numberOfKeyFrames; j++)
         {
            KinematicsToolboxRigidBodyMessage nextMessage = ikRigidBodyMessages.get(j);
            Pose3D nextDesiredPose = new Pose3D(nextMessage.getDesiredPositionInWorld(), nextMessage.getDesiredOrientationInWorld());
            for (int k = 0; k < numberOfInterpolatedPoints; k++)
            {
               double alpha = ((double) k + 1) / (numberOfInterpolatedPoints + 1);
               Pose3D interpolatedPose = new Pose3D(currentPose);
               interpolatedPose.interpolate(nextDesiredPose, alpha);
               ikRigidBodyPoses.add(interpolatedPose);
            }
            currentPose.set(nextDesiredPose);
            ikRigidBodyPoses.add(nextDesiredPose);
         }

         // re-create ik rigid body messages with the interpolated way points.
         KinematicsToolboxRigidBodyMessage rigidBodyMessageBuffer = new KinematicsToolboxRigidBodyMessage();
         rigidBodyMessageBuffer.set(ikRigidBodyMessages.get(0));
         ikRigidBodyMessages.clear();
         for (int j = 0; j < ikRigidBodyPoses.size(); j++)
         {
            KinematicsToolboxRigidBodyMessage messageToAdd = new KinematicsToolboxRigidBodyMessage(rigidBodyMessageBuffer);
            messageToAdd.getDesiredPositionInWorld().set(ikRigidBodyPoses.get(j).getPosition());
            messageToAdd.getDesiredOrientationInWorld().set(ikRigidBodyPoses.get(j).getOrientation());
            ikRigidBodyMessages.add(messageToAdd);
         }
      }

      // create interpolated way points for com.
      List<Point3D> comPoints = new ArrayList<Point3D>();
      CenterOfMassCalculator calculator = new CenterOfMassCalculator(getDesiredFullRobotModel().getRootBody(), worldFrame);
      calculator.reset();
      Point3D currentPoint = new Point3D(calculator.getCenterOfMass());
      for (int i = 0; i < ikCenterOfMassMessages.size(); i++)
      {
         Point3D nextDesiredPoint = new Point3D(ikCenterOfMassMessages.get(i).getDesiredPositionInWorld());
         for (int j = 0; j < numberOfInterpolatedPoints; j++)
         {
            double alpha = ((double) j + 1) / (numberOfInterpolatedPoints + 1);
            Point3D interpolatedPoint = new Point3D(currentPoint);
            interpolatedPoint.interpolate(nextDesiredPoint, alpha);
            comPoints.add(interpolatedPoint);
         }
         currentPoint.set(nextDesiredPoint);
         comPoints.add(nextDesiredPoint);
      }

      // re-create ik rigid body messages with the interpolated way points.
      KinematicsToolboxCenterOfMassMessage comMessageBuffer = new KinematicsToolboxCenterOfMassMessage();
      comMessageBuffer.set(ikCenterOfMassMessages.get(0));
      ikCenterOfMassMessages.clear();
      for (int j = 0; j < comPoints.size(); j++)
      {
         KinematicsToolboxCenterOfMassMessage messageToAdd = new KinematicsToolboxCenterOfMassMessage(comMessageBuffer);
         messageToAdd.getDesiredPositionInWorld().set(comPoints.get(j));
         ikCenterOfMassMessages.add(messageToAdd);
      }

      return true;
   }

   private boolean updateToolboxConfiguration()
   {
      if (rigidBodyCommands.size() == 0)
         throw new RuntimeException(this.getClass().getSimpleName() + " needs at least one KinematicsPlanningToolboxRigidBodyMessage.");

      ikRigidBodies.clear();
      ikRigidBodyMessageMap.clear();
      ikCenterOfMassMessages.clear();
      keyFrameTimes.clear();

      for (int i = 0; i < rigidBodyCommands.size(); i++)
      {
         KinematicsPlanningToolboxRigidBodyCommand command = rigidBodyCommands.get(i);

         RigidBodyBasics endEffector = command.getEndEffector();
         ikRigidBodies.add(endEffector);

         List<KinematicsToolboxRigidBodyMessage> rigidBodyMessages = new ArrayList<KinematicsToolboxRigidBodyMessage>();
         int numberOfWayPoints = command.getNumberOfWayPoints();
         for (int j = 0; j < numberOfWayPoints; j++)
         {
            // when the number of way point is less than 3, create and add mid point before the message for each way point is created.
            Pose3D wayPoint = command.getWayPoint(j);

            FramePose3D wayPointFramePose = new FramePose3D(worldFrame, wayPoint);

            KinematicsToolboxRigidBodyMessage rigidBodyMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(endEffector,
                                                                                                                      wayPointFramePose.getPosition(),
                                                                                                                      wayPointFramePose.getOrientation());
            rigidBodyMessage.getControlFramePositionInEndEffector().set(command.getControlFramePose().getPosition());
            rigidBodyMessage.getControlFrameOrientationInEndEffector().set(command.getControlFramePose().getOrientation());
            rigidBodyMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(command.getSelectionMatrix().getLinearPart()));
            rigidBodyMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(command.getSelectionMatrix().getAngularPart()));
            rigidBodyMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(command.getWeightMatrix().getLinearPart()));
            rigidBodyMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(command.getWeightMatrix().getAngularPart()));
            // TODO : allowable displacement.

            rigidBodyMessages.add(rigidBodyMessage);
         }
         ikRigidBodyMessageMap.put(endEffector, rigidBodyMessages);

         if (i == 0) // save key frame times for the first rigid body message.
         {
            for (int j = 0; j < numberOfWayPoints; j++)
               keyFrameTimes.add(command.getWayPointTime(j));
         }
         else if (!checkKeyFrameTimes(command.getWayPointTimes()))
         {
            throw new RuntimeException(command.getClass().getSimpleName()
                  + " must have same key frame times with the first KinematicsPlanningToolboxRigidBodyMessage.");
         }
      }

      if (centerOfMassCommand.get() != null && centerOfMassCommand.get().getNumberOfWayPoints() != 0)
      {
         for (int i = 0; i < centerOfMassCommand.get().getNumberOfWayPoints(); i++)
         {
            FramePoint3D wayPointFramePoint = new FramePoint3D(worldFrame, centerOfMassCommand.get().getWayPoint(i));
            KinematicsToolboxCenterOfMassMessage comMessage = MessageTools.createKinematicsToolboxCenterOfMassMessage(wayPointFramePoint);
            comMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(centerOfMassCommand.get().getWeightMatrix()));
            comMessage.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(centerOfMassCommand.get().getSelectionMatrix()));

            ikCenterOfMassMessages.add(comMessage);
         }

         if (!checkKeyFrameTimes(centerOfMassCommand.get().getWayPointTimes()))
            throw new RuntimeException(centerOfMassCommand.get().getClass().getSimpleName()
                  + " must have same key frame times with KinematicsPlanningToolboxRigidBodyMessage.");
      }
      else
      {
         for (int i = 0; i < getNumberOfKeyFrames(); i++)
         {
            getDesiredFullRobotModel().updateFrames();
            KinematicsToolboxCenterOfMassMessage comMessage = KinematicsToolboxMessageFactory.holdCenterOfMassCurrentPosition(getDesiredFullRobotModel().getRootBody(),
                                                                                                                              true, true, false);

            ikCenterOfMassMessages.add(comMessage);
         }
      }

      if (configurationCommand.get() != null)
      {

      }
      else
      {
         // TODO : implement please.
         //ikConfigurationMessage.set(KinematicsToolboxMessageFactory.privilegedConfigurationFromFullRobotModel(getDesiredFullRobotModel(), false));
      }

      return true;
   }

   @Override
   public void updateInternal() throws RuntimeException
   {
      if (solutionQualityConvergenceDetector.isSolved())
      {
         if (DEBUG)
            System.out.println("solved " + solutionQualityConvergenceDetector.isValid() + " " + solutionQualityConvergenceDetector.getNumberOfIteration());

         appendRobotConfigurationOnToolboxSolution();

         if (indexOfCurrentKeyFrame.getIntegerValue() == getNumberOfKeyFrames())
         {
            packSolution();
         }
         else
         {
            totalComputationTime.add(solutionQualityConvergenceDetector.getComputationTime());

            solutionQualityConvergenceDetector.initialize();
            submitKeyFrameMessages();
         }
      }
      else
      {
         ikController.updateInternal();
         solutionQualityConvergenceDetector.submitSolutionQuality(ikController.getSolution().getSolutionQuality());
         solutionQualityConvergenceDetector.update();
      }
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private void packSolution()
   {
      isDone.set(true);

      boolean isOptimalSolution = (solution.getPlanId() == KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_UNREACHABLE_KEYFRAME) ? false
            : true;

      generateTrajectoriesToPreview(false);
      if (isVelocityLimitExceeded() && useKeyFrameTimeOptimizerIfJointVelocityExceedLimits)
      {
         System.out.println("re planning for velocity optimization");
         generateTrajectoriesToPreview(true);
      }

      if (isVelocityLimitExceeded() && isOptimalSolution)
      {
         solution.setPlanId(KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_EXCEED_JOINT_VELOCITY_LIMIT);
         isOptimalSolution = false;
      }

      fullRobotModelTrajectoryCalculator.packOptimizedVelocities(solution);

      convertWholeBodyTrajectoryMessage();

      if (isOptimalSolution)
         solution.setPlanId(KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION);

      if (DEBUG)
         System.out.println("total computation time is " + solutionQualityConvergenceDetector.getComputationTime() + ", PlanId = " + solution.getPlanId());
      reportMessage(solution);
   }

   private void generateTrajectoriesToPreview(boolean useKeyFrameOptimizer)
   {
      fullRobotModelTrajectoryCalculator.clear();
      fullRobotModelTrajectoryCalculator.addKeyFrames(solution.getRobotConfigurations(), solution.getKeyFrameTimes());
      fullRobotModelTrajectoryCalculator.initializeCalculator();
      if (useKeyFrameOptimizer)
         fullRobotModelTrajectoryCalculator.computeOptimizingKeyFrameTimes();
      else
         fullRobotModelTrajectoryCalculator.computeForFixedKeyFrameTimes();
      fullRobotModelTrajectoryCalculator.computeVelocityBounds(searchingTimeTickForVelocityBound);
   }

   private boolean isVelocityLimitExceeded()
   {
      for (String armJointName : armJointNames)
      {
         double jointVelocityLowerBound = fullRobotModelTrajectoryCalculator.getJointVelocityLowerBound(armJointName);
         double jointVelocityUpperBound = fullRobotModelTrajectoryCalculator.getJointVelocityUpperBound(armJointName);

         Pair<Double, Double> velocityLimit = armJointVelocityLimitMap.get(armJointName);
         if (velocityLimit.i > jointVelocityLowerBound || velocityLimit.ii < jointVelocityUpperBound)
         {
            System.out.println(armJointName + " trajectory exceeds joint velocity limit." + " " + jointVelocityLowerBound + " " + jointVelocityUpperBound);
            return true;
         }
      }

      return false;
   }

   private void convertWholeBodyTrajectoryMessage()
   {
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      wholeBodyTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.computeWholeBodyTrajectoryMessage(solution);
      solution.getSuggestedControllerMessage().set(wholeBodyTrajectoryMessage);
   }

   private void appendRobotConfigurationOnToolboxSolution()
   {
      KinematicsToolboxOutputStatus keyFrame = new KinematicsToolboxOutputStatus(ikController.getSolution());
      solution.getRobotConfigurations().add().set(keyFrame);

      solution.setSolutionQuality(solution.getSolutionQuality() + ikController.getSolution().getSolutionQuality());

      if (!solutionQualityConvergenceDetector.isValid())
         solution.setPlanId(KinematicsPlanningToolboxOutputStatus.KINEMATICS_PLANNING_RESULT_UNREACHABLE_KEYFRAME);
   }

   private boolean submitKeyFrameMessages()
   {
      for (int i = 0; i < ikRigidBodies.size(); i++)
      {
         List<KinematicsToolboxRigidBodyMessage> rigidBodyMessages = ikRigidBodyMessageMap.get(ikRigidBodies.get(i));
         ikCommandInputManager.submitMessage(rigidBodyMessages.get(indexOfCurrentKeyFrame.getIntegerValue()));
      }

      if (ikCenterOfMassMessages.get(indexOfCurrentKeyFrame.getIntegerValue()) != null)
         ikCommandInputManager.submitMessage(ikCenterOfMassMessages.get(indexOfCurrentKeyFrame.getIntegerValue()));
      if (ikConfigurationMessage.get() != null)
         ikCommandInputManager.submitMessage(ikConfigurationMessage.get());

      indexOfCurrentKeyFrame.increment();

      return true;
   }

   private boolean updateInitialRobotConfigurationToIKController()
   {
      RobotConfigurationData currentRobotConfiguration = latestRobotConfigurationDataReference.getAndSet(null);
      if (currentRobotConfiguration == null)
      {
         LogTools.warn("latestRobotConfigurationDataReference should be set up.");
         return false;
      }

      FloatingJointBasics rootJoint = getDesiredFullRobotModel().getRootJoint();
      OneDoFJointBasics[] allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(getDesiredFullRobotModel());
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(currentRobotConfiguration, rootJoint, allJointsExcludingHands);
      MessageTools.packDesiredJointState(initialRobotConfiguration, rootJoint, allJointsExcludingHands);

      ikController.updateRobotConfigurationData(currentRobotConfiguration);

      CapturabilityBasedStatus capturabilityBasedStatus = latestCapturabilityBasedStatusReference.get();

      if (capturabilityBasedStatus == null)
      {
         LogTools.warn("capturabilityBasedStatus should be set up.");
         return false;
      }

      ikController.updateCapturabilityBasedStatus(capturabilityBasedStatus);
      return true;
   }

   private boolean checkKeyFrameTimes(TDoubleArrayList wayPointTimes)
   {
      if (wayPointTimes.size() != getNumberOfKeyFrames())
         return false;
      for (int i = 0; i < getNumberOfKeyFrames(); i++)
         if (!EuclidCoreTools.epsilonEquals(wayPointTimes.get(i), keyFrameTimes.get(i), keyFrameTimeEpsilon))
            return false;

      return true;
   }

   private int getNumberOfKeyFrames()
   {
      return keyFrameTimes.size();
   }

   public KinematicsPlanningToolboxOutputStatus getSolution()
   {
      return solution;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      latestCapturabilityBasedStatusReference.set(newStatus);
   }
}
