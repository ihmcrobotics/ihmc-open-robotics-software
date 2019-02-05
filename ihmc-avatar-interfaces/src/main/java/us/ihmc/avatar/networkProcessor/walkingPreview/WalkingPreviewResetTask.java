package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;

public class WalkingPreviewResetTask implements WalkingPreviewTask
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<WalkingPreviewContactStateHolder> contactStateHolders = new SideDependentList<>();
   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   private final CommandInputManager walkingInputManager;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private boolean haveResetCommandsBeenSubmitted = false;
   private final AtomicReference<RobotMotionStatus> latestMotionStatus = new AtomicReference<>(null);
   private RobotMotionStatusChangedListener robotMotionStatusChangedListener = (newStatus, time) -> latestMotionStatus.set(newStatus);

   private NeckTrajectoryMessage resetNeckMessage = null;
   private SideDependentList<ArmTrajectoryMessage> resetArmMessages = null;
   private ChestTrajectoryMessage resetChestMessage = null;
   private PelvisTrajectoryMessage resetPelvisMessage = null;

   private int countSinceMessageSent = 0;
   private int numberOfControlTicksBeforeDone = 10;

   public WalkingPreviewResetTask(SideDependentList<YoPlaneContactState> footContactStates, CommandInputManager walkingInputManager,
                                  HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.footContactStates = footContactStates;
      this.walkingInputManager = walkingInputManager;
      this.controllerToolbox = controllerToolbox;
   }

   public void resetToFullRobotModel(FullHumanoidRobotModel fullRobotModel)
   { // Get the controller to be initialized to hold the initial robot configuration.

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      { // Neck
         double[] desiredJointPositions = Stream.of(MultiBodySystemTools.createOneDoFJointPath(chest, head)).mapToDouble(OneDoFJointBasics::getQ).toArray();
         resetNeckMessage = HumanoidMessageTools.createNeckTrajectoryMessage(0.0, desiredJointPositions);
      }

      { // Chest
         FrameQuaternion initialChestOrientation = new FrameQuaternion(chest.getBodyFixedFrame());
         initialChestOrientation.changeFrame(worldFrame);
         resetChestMessage = HumanoidMessageTools.createChestTrajectoryMessage(0.0, initialChestOrientation, worldFrame);
      }

      resetArmMessages = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      { // Arms
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         double[] desiredJointPositions = Stream.of(MultiBodySystemTools.createOneDoFJointPath(chest, hand)).mapToDouble(joint -> joint.getQ()).toArray();
         resetArmMessages.put(robotSide, HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 0.0, desiredJointPositions));
      }

      { // Pelvis
         FramePose3D initialPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
         initialPelvisPose.changeFrame(worldFrame);
         resetPelvisMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(0.0, initialPelvisPose.getPosition(), initialPelvisPose.getOrientation());
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
         contactStateHolders.put(robotSide, WalkingPreviewContactStateHolder.holdAtCurrent(footContactStates.get(robotSide)));
      controllerToolbox.attachRobotMotionStatusChangedListener(robotMotionStatusChangedListener);
   }

   private void snapToInitialRobotConfiguration()
   { // Get the controller to be initialized to hold the initial robot configuration.
      walkingInputManager.submitMessage(resetPelvisMessage);
      walkingInputManager.submitMessage(resetNeckMessage);
      walkingInputManager.submitMessage(resetChestMessage);
      for (RobotSide robotSide : RobotSide.values)
         walkingInputManager.submitMessage(resetArmMessages.get(robotSide));
   }

   @Override
   public void doAction()
   {
      if (!haveResetCommandsBeenSubmitted && latestMotionStatus.get() == RobotMotionStatus.STANDING)
      {
         snapToInitialRobotConfiguration();
         haveResetCommandsBeenSubmitted = true;
      }

      commandList.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         contactStateHolders.get(robotSide).doControl();
         commandList.addCommand(contactStateHolders.get(robotSide).getOutput());
      }

      if (haveResetCommandsBeenSubmitted)
         countSinceMessageSent++;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      destroyListeners();
   }

   @Override
   public boolean isDone()
   {
      return countSinceMessageSent >= numberOfControlTicksBeforeDone;
   }

   @Override
   public InverseDynamicsCommand<?> getOutput()
   {
      return commandList;
   }

   @Override
   protected void finalize() throws Throwable
   {
      super.finalize();
      destroyListeners(); // In case the doTransitionOutOfAction() was not called somehow.
   }

   private void destroyListeners()
   {
      if (robotMotionStatusChangedListener != null)
      {
         controllerToolbox.detachRobotMotionStatusChangedListener(robotMotionStatusChangedListener);
         robotMotionStatusChangedListener = null;
      }
   }
}
