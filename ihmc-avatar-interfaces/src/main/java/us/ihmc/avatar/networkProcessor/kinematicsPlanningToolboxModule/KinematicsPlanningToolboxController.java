package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

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
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsPlanningToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class KinematicsPlanningToolboxController extends ToolboxController
{
   private final boolean DEBUG = true;

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager commandInputManager;

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

   private final static double keyFrameTimeEpsilon = 0.01;
   private final TDoubleArrayList keyFrameTimes;

   private final HumanoidKinematicsToolboxController ikController;
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

      ikController.updateFootSupportState(true, true);
      boolean initialized = ikController.initialize();
      if (!initialized)
         throw new RuntimeException("Could not initialize the " + KinematicsToolboxController.class.getSimpleName());

      solution.getKeyFrameTimes().clear();
      solution.getRobotConfigurations().clear();
      solution.setSolutionQuality(0.0);

      for (int i = 0; i < getNumberOfKeyFrames(); i++)
         solution.getKeyFrameTimes().add(keyFrameTimes.get(i));

      solutionQualityConvergenceDetector.initialize();
      submitKeyFrameMessages();

      if (DEBUG)
         System.out.println("Initializing is done");
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

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(getDesiredFullRobotModel());
      referenceFrames.updateFrames();
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFootZUpGroundFrame();

      for (int i = 0; i < rigidBodyCommands.size(); i++)
      {
         KinematicsPlanningToolboxRigidBodyCommand command = rigidBodyCommands.get(i);

         RigidBodyBasics endEffector = command.getEndEffector();
         ikRigidBodies.add(endEffector);

         List<KinematicsToolboxRigidBodyMessage> rigidBodyMessages = new ArrayList<KinematicsToolboxRigidBodyMessage>();
         for (int j = 0; j < command.getNumberOfWayPoints(); j++)
         {
            Pose3D wayPoint = command.getWayPoint(j);

            FramePose3D wayPointFramePose = new FramePose3D(midFeetZUpFrame, wayPoint);

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
            for (int j = 0; j < command.getNumberOfWayPoints(); j++)
               keyFrameTimes.add(command.getWayPointTime(j));
         }
         else if (!checkKeyFrameTimes(command))
         {
            throw new RuntimeException(command.getClass().getSimpleName()
                  + " must have same key frame times with the first KinematicsPlanningToolboxRigidBodyMessage.");
         }
      }

      if (centerOfMassCommand.get() != null && centerOfMassCommand.get().getNumberOfWayPoints() != 0)

      {
         if (!checkKeyFrameTimes(centerOfMassCommand.get()))
            throw new RuntimeException(centerOfMassCommand.get().getClass().getSimpleName()
                  + " must have same key frame times with KinematicsPlanningToolboxRigidBodyMessage.");

         for (int i = 0; i < centerOfMassCommand.get().getNumberOfWayPoints(); i++)
         {
            KinematicsToolboxCenterOfMassMessage comMessage = MessageTools.createKinematicsToolboxCenterOfMassMessage(centerOfMassCommand.get().getWayPoint(i));
            comMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(centerOfMassCommand.get().getWeightMatrix()));
            comMessage.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(centerOfMassCommand.get().getSelectionMatrix()));

            ikCenterOfMassMessages.add(comMessage);
         }
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

   private FramePoint3DReadOnly computeCenterOfMass3D(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      CenterOfMassCalculator calculator = new CenterOfMassCalculator(fullHumanoidRobotModel.getElevator(), ReferenceFrame.getWorldFrame());
      return calculator.getCenterOfMass();
   }

   @Override
   public void updateInternal() throws RuntimeException
   {
      if (solutionQualityConvergenceDetector.isSolved())
      {
         if (DEBUG)
            System.out.println("solved " + solutionQualityConvergenceDetector.isValid() + " " + solutionQualityConvergenceDetector.getNumberOfIteration());
         if (!appendRobotConfigurationOnToolboxSolution() || indexOfCurrentKeyFrame.getIntegerValue() == getNumberOfKeyFrames())
         {
            isDone.set(true);
            solution.setDestination(PacketDestination.BEHAVIOR_MODULE.ordinal());
            WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
            wholeBodyTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
            outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
            outputConverter.computeWholeBodyTrajectoryMessage(solution);
            solution.getSuggestedControllerMessage().set(wholeBodyTrajectoryMessage);

            if (DEBUG)
               System.out.println("total computation time is " + solutionQualityConvergenceDetector.getComputationTime());
            reportMessage(solution);
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

   private boolean appendRobotConfigurationOnToolboxSolution()
   {
      if (solutionQualityConvergenceDetector.isValid())
      {
         solution.setSolutionQuality(solution.getSolutionQuality() + ikController.getSolution().getSolutionQuality());
         solution.getRobotConfigurations().add().set(new KinematicsToolboxOutputStatus(ikController.getSolution()));

         return true;
      }
      else
      {
         solution.setSolutionQuality(-1);
         return false;
      }
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

      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(currentRobotConfiguration, getDesiredFullRobotModel().getRootJoint(),
                                                                      FullRobotModelUtils.getAllJointsExcludingHands(getDesiredFullRobotModel()));

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

   private boolean checkKeyFrameTimes(KinematicsPlanningToolboxRigidBodyCommand command)
   {
      if (command.getNumberOfWayPoints() != getNumberOfKeyFrames())
         return false;
      for (int i = 0; i < getNumberOfKeyFrames(); i++)
         if (!EuclidCoreTools.epsilonEquals(command.getWayPointTime(i), keyFrameTimes.get(i), keyFrameTimeEpsilon))
            return false;

      return true;
   }

   private boolean checkKeyFrameTimes(KinematicsPlanningToolboxCenterOfMassCommand command)
   {
      if (command.getNumberOfWayPoints() != getNumberOfKeyFrames())
         return false;
      for (int i = 0; i < getNumberOfKeyFrames(); i++)
         if (!EuclidCoreTools.epsilonEquals(command.getWayPointTime(i), keyFrameTimes.get(i), keyFrameTimeEpsilon))
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
