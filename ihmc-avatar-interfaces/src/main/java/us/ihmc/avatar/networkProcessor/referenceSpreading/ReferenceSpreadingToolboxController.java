package us.ihmc.avatar.networkProcessor.referenceSpreading;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import controller_msgs.msg.dds.*;
import gnu.trove.map.hash.TIntObjectHashMap;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.referenceSpreading.ReferenceSpreadingStateHelper.RSTimeProvider;
import us.ihmc.avatar.networkProcessor.referenceSpreading.ReferenceSpreadingStateHelper.States;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class ReferenceSpreadingToolboxController extends ToolboxController
{
   private final YoBoolean waitingForRobotConfigurationData = new YoBoolean("waitingForRobotConfigurationData", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();

   private final RSTimeProvider timeProvider;
   private final YoDouble time = new YoDouble("time", registry);
   StateMachine<States, State> stateMachine;
   ReferenceSpreadingStateHelper stateMachineHelper;

   private final HumanoidReferenceFrames referenceFrames;
   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();

   private HandTrajectoryMessagePublisher trajectoryMessagePublisher = m-> {LogTools.error("No publisher set");};

   public ReferenceSpreadingToolboxController(DRCRobotModel robotModel,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   int updateRateMillis,
                                                   YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      timeProvider = RSTimeProvider.createTimeProvider();

      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator())
                          .subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);

//      todo: Add proper way of importing the CSV file.
      String demoDirectory = Objects.requireNonNull(new WorkspaceDirectory("nadia",
                                                                           "nadia-hardware-drivers/src/test/resources/hybridPlaybackCSVs").getFilesystemDirectory()).toString();
//      String filePath = demoDirectory + "/testCSV.csv";
      String filePath = demoDirectory + "/boxPickup.csv";

      stateMachineHelper = new ReferenceSpreadingStateHelper(filePath, fullRobotModel, trajectoryMessagePublisher, registry);
      stateMachine = stateMachineHelper.setUpStateMachines(time);
   }

   @Override
   public boolean initialize()
   {
      waitingForRobotConfigurationData.set(true);
      isDone.set(false);

      timeProvider.initialize();
      time.set(timeProvider.getTime());

      return true;
   }

   @Override
   public void updateInternal()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         referenceFrames.updateFrames();
         waitingForRobotConfigurationData.set(false);
      }
      else
      {
         waitingForRobotConfigurationData.set(true);
      }

      if (waitingForRobotConfigurationData.getBooleanValue())
         return;

      timeProvider.update(robotConfigurationData.getMonotonicTime());
      time.set(timeProvider.getTime());
      stateMachineHelper.updateJointVelocities(robotConfigurationData.getJointVelocities());

      stateMachine.doActionAndTransition();

//      LogTools.info("/dot{q} = " + robotConfigurationData.getJointVelocities());
   }

   public void resetToInitialState()
   {
      stateMachine.resetToInitialState();
   }

   public void goToWaiting()
   {
      stateMachine.performTransition(States.WAITING);
   }

   public void startRS()
   {
      stateMachine.performTransition(States.BEFORE);
   }

   public void setTrajectoryMessagePublisher(HandTrajectoryMessagePublisher trajectoryMessagePublisher)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
      stateMachineHelper.setTrajectoryMessagePublisher(trajectoryMessagePublisher);
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      this.capturabilityBasedStatus.set(capturabilityBasedStatus);
      stateMachineHelper.updateHandWrenches(capturabilityBasedStatus);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public interface HandTrajectoryMessagePublisher
   {
      void publish(HandHybridJointspaceTaskspaceTrajectoryMessage messageToPublish);
   }
}
