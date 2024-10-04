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

   private final RSTimeProvider timeProvider;
   private final YoDouble time = new YoDouble("time", registry);
   StateMachine<States, State> stateMachine;

   private final HumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();

   private HandTrajectoryMessagePublisher trajectoryMessagePublisher;

   public ReferenceSpreadingToolboxController(DRCRobotModel robotModel,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   int updateRateMillis,
                                                   YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      timeProvider = RSTimeProvider.createTimeProfider();

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
      this.rootJoint = fullRobotModel.getRootJoint();

      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator())
                          .subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);

      double updateDT = Conversions.millisecondsToSeconds(updateRateMillis);
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(robotModel.getControllerDT(),
                                                                                       9.81,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       joints,
                                                                                       referenceFrames.getCenterOfMassFrame(),
                                                                                       robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings(),
                                                                                       graphicsListRegistry,
                                                                                       parentRegistry);

      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }
      controlCoreToolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
      List<Collidable> collidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());

      String demoDirectory = Objects.requireNonNull(new WorkspaceDirectory("nadia",
                                                                           "nadia-hardware-drivers/src/test/resources/hybridPlaybackCSVs").getFilesystemDirectory()).toString();
      String filePath = demoDirectory + "/pickUpBox.csv";

      ReferenceSpreadingStateHelper stateMachineHelper = new ReferenceSpreadingStateHelper(filePath, trajectoryMessagePublisher, registry);
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

      if (waitingForRobotConfigurationData.getBooleanValue())
         return;



      timeProvider.update(robotConfigurationData.getMonotonicTime());
      time.set(timeProvider.getTime());

      stateMachine.doActionAndTransition();
   }

   public void setTrajectoryMessagePublisher(HandTrajectoryMessagePublisher trajectoryMessagePublisher)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public HandTrajectoryMessage createHandTrajectoryMessage(HashMap<String, Double> thisFrame, RobotSide robotSide)
   {
      SE3TrajectoryPointMessage se3TrajectoryPointMessage = new SE3TrajectoryPointMessage();


      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(robotSide.toByte());
      handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add(se3TrajectoryPointMessage);
      return handTrajectoryMessage;
   }

   public interface HandTrajectoryMessagePublisher
   {
      void publish(HandTrajectoryMessage messageToPublish);
   }
}
