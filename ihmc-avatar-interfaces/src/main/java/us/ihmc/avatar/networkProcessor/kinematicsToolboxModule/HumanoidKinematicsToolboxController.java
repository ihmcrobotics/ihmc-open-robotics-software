package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class HumanoidKinematicsToolboxController extends KinematicsToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /**
    * This is the model of the robot that is constantly updated to represent the most recent
    * solution obtained. The {@link WholeBodyControllerCore} works on this robot to perform the
    * feedback controllers.
    */
   private final FullHumanoidRobotModel desiredFullRobotModel;

   /**
    * Updated during the initialization phase, this set of two {@link YoBoolean}s is used to know
    * which foot is currently used for support in the walking controller.
    */
   private final SideDependentList<YoBoolean> isFootInSupport = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the poses of the feet are stored so
    * they can be held in place during the optimization process such that the solution will be
    * statically reachable.
    */
   private final SideDependentList<YoFramePose3D> initialFootPoses = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the robot's center of mass position is
    * stored so it can be held in place during the optimization process such that the solution will
    * be statically reachable.
    */
   private final YoFramePoint3D initialCenterOfMassPosition = new YoFramePoint3D("initialCenterOfMass", worldFrame, registry);

   /**
    * Indicates whether the support foot/feet should be held in place for this run. It is
    * {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdSupportFootPose = new YoBoolean("holdSupportFootPose", registry);
   /**
    * Indicates whether the center of mass x and y coordinates should be held in place for this run.
    * It is {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdCenterOfMassXYPosition = new YoBoolean("holdCenterOfMassXYPosition", registry);
   /**
    * Default weight used when holding the support foot/feet in place. It is rather high such that
    * they do not deviate much from their initial poses.
    */
   private final YoDouble footWeight = new YoDouble("footWeight", registry);
   /**
    * Default weight used when holding the center of mass in place. It is rather high such that it
    * does not deviate much from its initial position.
    */
   private final YoDouble momentumWeight = new YoDouble("momentumWeight", registry);
   /**
    * This joint reduction factors are used to limit the range of motion for each joint in the
    * controller core. The formulated based on the percentage of the actual range of motion for each
    * joint such that a factor of 0.05 for the hip yaw will effectively reduce the allowed range of
    * motion by 2.5% on the upper and lower end of the joint.
    */
   private final EnumMap<LegJointName, YoDouble> legJointLimitReductionFactors = new EnumMap<>(LegJointName.class);
   /**
    * Reference to the most recent data received from the controller relative to the balance
    * control. It is used for identifying which foot is in support and thus which foot should be
    * held in place.
    */
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoVariableRegistry parentRegistry)
   {
      super(commandInputManager, statusOutputManager, desiredFullRobotModel.getRootJoint(), getAllJointsExcludingHands(desiredFullRobotModel),
            createListOfControllableRigidBodies(desiredFullRobotModel), yoGraphicsListRegistry, parentRegistry);

      this.desiredFullRobotModel = desiredFullRobotModel;

      footWeight.set(200.0);
      momentumWeight.set(1.0);

      for (RobotSide robotSide : RobotSide.values)
         setupVisualization(desiredFullRobotModel.getHand(robotSide), desiredFullRobotModel.getFoot(robotSide));

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         isFootInSupport.put(robotSide, new YoBoolean("is" + side + "FootInSupport", registry));
         initialFootPoses.put(robotSide, new YoFramePose3D(sidePrefix + "FootInitial", worldFrame, registry));
      }

      populateJointLimitReductionFactors();
   }

   /**
    * Setting up the map holding the joint limit reduction factors. If more reduction is needed, add
    * it there. If it has to be updated on the fly, it should then be added this toolbox API,
    * probably added to the message {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private void populateJointLimitReductionFactors()
   {
      YoDouble hipReductionFactor = new YoDouble("hipLimitReductionFactor", registry);
      YoDouble kneeReductionFactor = new YoDouble("kneeLimitReductionFactor", registry);
      YoDouble ankleReductionFactor = new YoDouble("ankleLimitReductionFactor", registry);
      hipReductionFactor.set(0.05);

      legJointLimitReductionFactors.put(LegJointName.HIP_PITCH, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_ROLL, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_YAW, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.KNEE_PITCH, kneeReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_PITCH, ankleReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_ROLL, ankleReductionFactor);
   }

   private static Collection<RigidBody> createListOfControllableRigidBodies(FullHumanoidRobotModel desiredFullRobotModel)
   {
      List<RigidBody> listOfControllableRigidBodies = new ArrayList<>();

      listOfControllableRigidBodies.add(desiredFullRobotModel.getChest());
      listOfControllableRigidBodies.add(desiredFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         listOfControllableRigidBodies.add(desiredFullRobotModel.getHand(robotSide));
         listOfControllableRigidBodies.add(desiredFullRobotModel.getFoot(robotSide));
      }
      
      listOfControllableRigidBodies.add(desiredFullRobotModel.getHead());

      return listOfControllableRigidBodies;
   }

   @Override
   protected boolean initialize()
   {
      if (!super.initialize())
         return false;

      // Using the most recent CapturabilityBasedStatus received from the walking controller to figure out which foot is in support.
      CapturabilityBasedStatus capturabilityBasedStatus = latestCapturabilityBasedStatusReference.get();

      if (capturabilityBasedStatus == null)
      {
         for (RobotSide robotSide : RobotSide.values)
            isFootInSupport.get(robotSide).set(true);
      }
      else
      {
         for (RobotSide robotside : RobotSide.values)
            isFootInSupport.get(robotside).set(HumanoidMessageTools.unpackIsSupportFoot(capturabilityBasedStatus, robotside));
      }

      // Initialize the initialCenterOfMassPosition and initialFootPoses to match the current state of the robot.
      updateCoMPositionAndFootPoses();

      // By default, always hold the support foot/feet and center of mass in place. This can be changed on the fly by sending a KinematicsToolboxConfigurationMessage.
      holdSupportFootPose.set(true);
      holdCenterOfMassXYPosition.set(true);

      return true;
   }

   @Override
   protected void updateInternal()
   {
      if (commandInputManager.isNewCommandAvailable(HumanoidKinematicsToolboxConfigurationCommand.class))
      {
         HumanoidKinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(HumanoidKinematicsToolboxConfigurationCommand.class);

         holdCenterOfMassXYPosition.set(command.holdCurrentCenterOfMassXYPosition());
         holdSupportFootPose.set(command.holdSupportFootPositions());
      }

      super.updateInternal();
   }

   /**
    * Sets the {@link #initialCenterOfMassPosition} and {@link #initialFootPoses} to match the
    * current state of {@link #desiredFullRobotModel}.
    */
   private void updateCoMPositionAndFootPoses()
   {
      updateTools();

      initialCenterOfMassPosition.setFromReferenceFrame(centerOfMassFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         initialFootPoses.get(robotSide).setFromReferenceFrame(foot.getBodyFixedFrame());
      }
   }

   /**
    * Creates and sets up the feedback control commands for holding the support foot/feet in place.
    * If {@link #holdSupportFootPose} is {@code false}, this methods returns {@code null}.
    * <p>
    * Also note that if a user command has been received for a support foot, the command for this
    * foot is not created.
    * </p>
    *
    * @return the commands for holding the support foot/feet in place.
    */
   private FeedbackControlCommand<?> createHoldSupportFootCommands()
   {
      if (!holdSupportFootPose.getBooleanValue())
         return null;

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isFootInSupport.get(robotSide).getBooleanValue())
            continue;

         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);

         // Do not hold the foot position if the user is already controlling it.
         if (isUserControllingRigidBody(foot))
            continue;

         FramePose3D poseToHold = new FramePose3D(initialFootPoses.get(robotSide));

         SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, foot);
         feedbackControlCommand.setGains(getDefaultGains());
         feedbackControlCommand.setWeightForSolver(footWeight.getDoubleValue());
         feedbackControlCommand.set(poseToHold);
         inputs.addCommand(feedbackControlCommand);
      }
      return inputs;
   }

   /**
    * Creates and sets up the feedback control command for holding the center of mass x and y
    * coordinates in place. If {@link #holdCenterOfMassXYPosition} is {@code false}, this methods
    * returns {@code null}.
    * <p>
    * Also note that if a user command has been received for the center of mass, this methods
    * returns {@code null}.
    * </p>
    *
    * @return the commands for holding the center of mass x and y coordinates in place.
    */
   private FeedbackControlCommand<?> createHoldCenterOfMassXYCommand()
   {
      if (!holdCenterOfMassXYPosition.getBooleanValue())
         return null;

      // Do not hold the CoM position if the user is already controlling it.
      if (isUserControllingCenterOfMass())
      {
         holdCenterOfMassXYPosition.set(false);
         return null;
      }

      FramePoint3D positionToHold = new FramePoint3D(initialCenterOfMassPosition);

      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(getDefaultGains().getPositionGains());
      feedbackControlCommand.setWeightForSolver(momentumWeight.getDoubleValue());
      feedbackControlCommand.setSelectionMatrixForLinearXYControl();
      feedbackControlCommand.set(positionToHold);
      return feedbackControlCommand;
   }

   /**
    * Creates and sets up the {@code JointLimitReductionCommand} from the map
    * {@link #legJointLimitReductionFactors}.
    *
    * @return the command for reducing the allowed range of motion of the leg joints.
    */
   private JointLimitReductionCommand createJointLimitReductionCommand()
   {
      JointLimitReductionCommand jointLimitReductionCommand = new JointLimitReductionCommand();
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : desiredFullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJoint joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }
      return jointLimitReductionCommand;
   }

   @Override
   protected void robotConfigurationReinitialized()
   {
      updateCoMPositionAndFootPoses();
   }

   public void updateFootSupportState(boolean isLeftFootInSupport, boolean isRightFootInSupport)
   {
      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
      if (isLeftFootInSupport)
         capturabilityBasedStatus.getLeftFootSupportPolygon2d().add();
      if (isRightFootInSupport)
         capturabilityBasedStatus.getRightFootSupportPolygon2d().add();
      updateCapturabilityBasedStatus(capturabilityBasedStatus);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      latestCapturabilityBasedStatusReference.set(newStatus);
   }

   @Override
   protected FeedbackControlCommandList getAdditionalFeedbackControlCommands()
   {
      FeedbackControlCommandList commands = new FeedbackControlCommandList();
      commands.addCommand(createHoldSupportFootCommands());
      commands.addCommand(createHoldCenterOfMassXYCommand());
      return commands;
   }

   @Override
   protected InverseKinematicsCommandList getAdditionalInverseKinematicsCommands()
   {
      InverseKinematicsCommandList commands = new InverseKinematicsCommandList();
      commands.addCommand(createJointLimitReductionCommand());
      return commands;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }
}
