package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL;
import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.MultiContactBalanceStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Integer;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class HumanoidKinematicsToolboxController extends KinematicsToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /**
    * This is the model of the robot that is constantly updated to represent the most recent solution
    * obtained. The {@link WholeBodyControllerCore} works on this robot to perform the feedback
    * controllers.
    */
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommonHumanoidReferenceFrames desiredReferenceFrames;
   /**
    * Robot model used to represent the current robot configuration as measured from
    * {@link RobotConfigurationData}.
    */
   private final FullHumanoidRobotModel currentFullRobotModel;
   private final CommonHumanoidReferenceFrames currentReferenceFrames;
   private final OneDoFJointBasics[] currentOneDoFJoints;
   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashCodeMap = new TIntObjectHashMap<>();

   private final Map<RigidBodyBasics, RigidBodyBasics> endEffectorToPrimaryBaseMap = new HashMap<>();

   private final YoBoolean enableAutoSupportPolygon = new YoBoolean("enableAutoSupportPolygon", registry);
   /**
    * Updated during the initialization phase with {@link CapturabilityBasedStatus}, this set of two
    * {@link YoBoolean}s is used to know which foot is currently used for support in the walking
    * controller.
    */
   private final SideDependentList<YoBoolean> isFootInSupport = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the poses of the feet are stored so they
    * can be held in place during the optimization process such that the solution will be statically
    * reachable.
    */
   private final SideDependentList<YoFramePose3D> initialFootPoses = new SideDependentList<>();
   /**
    * Updated during the initialization phase with {@link MultiContactBalanceStatus}, this list
    * contains all the necessary information about rigid-bodies currently used for support and for
    * controlling them.
    */
   private final RecyclingArrayList<ContactingRigidBody> contactingRigidBodies = new RecyclingArrayList<>(ContactingRigidBody::new);
   /**
    * Updated during the initialization phase, this is where the robot's center of mass position is
    * stored so it can be held in place during the optimization process such that the solution will be
    * statically reachable.
    */
   private final YoFramePoint3D initialCenterOfMassPosition = new YoFramePoint3D("initialCenterOfMass", worldFrame, registry);

   /**
    * Indicates whether the rigid-bodies currently in contact as reported per:
    * {@link CapturabilityBasedStatus} or {@link MultiContactBalanceStatus} should be held in place for
    * this run. It is {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdSupportRigidBodies = new YoBoolean("holdSupportRigidBodies", registry);
   /**
    * Indicates whether the center of mass x and y coordinates should be held in place for this run. It
    * is {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdCenterOfMassXYPosition = new YoBoolean("holdCenterOfMassXYPosition", registry);
   private final FramePoint3D centerOfMassPositionToHold = new FramePoint3D();
   /**
    * Default weight used when holding a support rigid-body in place. It is rather high such that they
    * do not deviate much from their initial position/pose.
    */
   private final YoDouble supportRigidBodyWeight = new YoDouble("supportRigidBodyWeight", registry);
   /**
    * Default weight used when holding the center of mass in place. It is rather high such that it does
    * not deviate much from its initial position.
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
    * Reference to the most recent data received from the controller relative to the balance control.
    * It is used for identifying which foot is in support and thus which foot should be held in place.
    */
   private final ConcurrentCopier<CapturabilityBasedStatus> concurrentCapturabilityBasedStatusCopier = new ConcurrentCopier<>(CapturabilityBasedStatus::new);
   private boolean hasCapturabilityBasedStatus = false;
   private final CapturabilityBasedStatus capturabilityBasedStatusInternal = new CapturabilityBasedStatus();

   /**
    * Reference to the most recent data received from the controller relative to the balance control.
    * It is used for identifying which rigid-body is used for support and thus which rigid-body should
    * be held in place.
    */
   private final ConcurrentCopier<MultiContactBalanceStatus> concurrentMultiContactBalanceStatusCopier = new ConcurrentCopier<>(MultiContactBalanceStatus::new);
   private boolean hasMultiContactBalanceStatus = false;
   private final MultiContactBalanceStatus multiContactBalanceStatusInternal = new MultiContactBalanceStatus();

   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                              double updateDT, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this(commandInputManager, statusOutputManager, desiredFullRobotModel, createListOfControllableRigidBodies(desiredFullRobotModel), fullRobotModelFactory,
           updateDT, yoGraphicsListRegistry, parentRegistry);
   }

   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel, Collection<? extends RigidBodyBasics> controllableRigidBodyies,
                                              FullHumanoidRobotModelFactory fullRobotModelFactory, double updateDT,
                                              YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      super(commandInputManager, statusOutputManager, desiredFullRobotModel.getRootJoint(), getAllJointsExcludingHands(desiredFullRobotModel),
            controllableRigidBodyies, updateDT, yoGraphicsListRegistry, parentRegistry);

      this.desiredFullRobotModel = desiredFullRobotModel;
      desiredReferenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      currentFullRobotModel = fullRobotModelFactory.createFullRobotModel();
      currentOneDoFJoints = getAllJointsExcludingHands(currentFullRobotModel);
      currentReferenceFrames = new HumanoidReferenceFrames(currentFullRobotModel);
      desiredFullRobotModel.getElevator().subtreeStream().forEach(rigidBody -> rigidBodyHashCodeMap.put(rigidBody.hashCode(), rigidBody));

      supportRigidBodyWeight.set(200.0);
      momentumWeight.set(0.001);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (desiredFullRobotModel.getHand(robotSide) != null)
            setupVisualization(desiredFullRobotModel.getHand(robotSide));
         setupVisualization(desiredFullRobotModel.getFoot(robotSide));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         isFootInSupport.put(robotSide, new YoBoolean("is" + side + "FootInSupport", registry));
         initialFootPoses.put(robotSide, new YoFramePose3D(sidePrefix + "FootInitial", worldFrame, registry));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         endEffectorToPrimaryBaseMap.put(desiredFullRobotModel.getChest(), desiredFullRobotModel.getHand(robotSide));
         endEffectorToPrimaryBaseMap.put(desiredFullRobotModel.getPelvis(), desiredFullRobotModel.getFoot(robotSide));
      }

      populateJointLimitReductionFactors();
   }

   /**
    * Setting up the map holding the joint limit reduction factors. If more reduction is needed, add it
    * there. If it has to be updated on the fly, it should then be added this toolbox API, probably
    * added to the message {@link HumanoidKinematicsToolboxConfigurationMessage}.
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

   private static Collection<RigidBodyBasics> createListOfControllableRigidBodies(FullHumanoidRobotModel desiredFullRobotModel)
   {
      List<RigidBodyBasics> listOfControllableRigidBodies = new ArrayList<>();

      listOfControllableRigidBodies.add(desiredFullRobotModel.getHead());
      listOfControllableRigidBodies.add(desiredFullRobotModel.getChest());
      listOfControllableRigidBodies.add(desiredFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         listOfControllableRigidBodies.add(desiredFullRobotModel.getHand(robotSide));
         listOfControllableRigidBodies.add(desiredFullRobotModel.getFoot(robotSide));
      }

      // Some robots may not have some the bodies.
      listOfControllableRigidBodies.removeIf(Objects::isNull);

      return listOfControllableRigidBodies;
   }

   /**
    * Convenience method for setting up the initial robot configuration using
    * {@link DRCRobotModel#getDefaultRobotInitialSetup(double, double)}.
    *
    * @param robotModel the robot model used to configure the initial configuration. The robot model
    *                   should match the robot used in this solver.
    */
   public void setInitialRobotConfiguration(DRCRobotModel robotModel)
   {
      Map<OneDoFJointBasics, Double> privilegedConfiguration = new HashMap<>();
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      defaultRobotInitialSetup.initializeRobot(robot, robotModel.getJointMap());

      for (OneDoFJointBasics joint : getDesiredOneDoFJoint())
      {
         double q_priv = robot.getOneDegreeOfFreedomJoint(joint.getName()).getQ();
         privilegedConfiguration.put(joint, q_priv);
      }

      setInitialRobotConfiguration(privilegedConfiguration);
   }

   public void setCollisionModel(RobotCollisionModel collisionModel)
   {
      if (collisionModel != null)
         registerCollidables(collisionModel.getRobotCollidables(getDesiredFullRobotModel().getElevator()));
   }

   @Override
   public boolean initialize()
   {
      KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
      status.setJointNameHash(-1);
      status.setSolutionQuality(Double.NaN);

      if (!super.initializeInternal())
      {
         status.setCurrentToolboxState(CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD);
         reportMessage(status);
         return false;
      }

      /*
       * Initialize the support conditions. There 2 scenarios: either the walking controller is running
       * and we use the CapturabilityBasedStatus to identify which foot is in support, or a multi-contact
       * controller is running and we use the MultiContactBalanceStatus to identify supporting
       * rigid-bodies and the info necessary to hold the contact points in place.
       */
      CapturabilityBasedStatus capturabilityBasedStatus = concurrentCapturabilityBasedStatusCopier.getCopyForReading();
      hasCapturabilityBasedStatus = capturabilityBasedStatus != null;
      if (hasCapturabilityBasedStatus)
         capturabilityBasedStatusInternal.set(capturabilityBasedStatus);

      MultiContactBalanceStatus multiContactBalanceStatus = concurrentMultiContactBalanceStatusCopier.getCopyForReading();
      hasMultiContactBalanceStatus = multiContactBalanceStatus != null;
      if (hasMultiContactBalanceStatus)
         multiContactBalanceStatusInternal.set(multiContactBalanceStatus);

      contactingRigidBodies.clear();

      if (hasCapturabilityBasedStatus)
      {
         for (RobotSide robotside : RobotSide.values)
            isFootInSupport.get(robotside).set(HumanoidMessageTools.unpackIsSupportFoot(capturabilityBasedStatusInternal, robotside));

         hasMultiContactBalanceStatus = false;
      }
      else if (hasMultiContactBalanceStatus)
      {
         Object<Point3D> supportPolygon = multiContactBalanceStatus.getSupportPolygon();
         Integer supportRigidBodyIds = multiContactBalanceStatus.getSupportRigidBodyIds();

         for (int i = 0; i < supportPolygon.size(); i++)
         {
            ContactingRigidBody contactingRigidBody = contactingRigidBodies.add();
            contactingRigidBody.initialize(rigidBodyHashCodeMap.get(supportRigidBodyIds.get(i)), worldFrame, supportPolygon.get(i));
         }

         for (RobotSide robotSide : RobotSide.values)
            isFootInSupport.get(robotSide).set(false);
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
            isFootInSupport.get(robotSide).set(true);
      }

      if (initialRobotConfigurationMap != null)
      {
         if (hasMultiContactBalanceStatus)
            throw new UnsupportedOperationException("Initial robot configuration is not supported with multi-contact context.");

         /*
          * Default initial configuration was provided and is set in the super class. The goal here, is to
          * recompute the pose of the root joint such that our initial configuration has its support feet as
          * close as possible to the current robot support feet. This affects the CoM task.
          */
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationDataInternal,
                                                                         currentFullRobotModel.getRootJoint(),
                                                                         currentOneDoFJoints);
         currentReferenceFrames.updateFrames();
         rootJoint.getJointPose().setToZero();
         desiredReferenceFrames.updateFrames();

         MovingReferenceFrame currentFrame, desiredFrame;

         if (isFootInSupport.get(RobotSide.LEFT).getValue())
         {
            if (isFootInSupport.get(RobotSide.RIGHT).getValue())
            {
               currentFrame = currentReferenceFrames.getMidFootZUpGroundFrame();
               desiredFrame = desiredReferenceFrames.getMidFootZUpGroundFrame();
            }
            else
            {
               currentFrame = currentReferenceFrames.getSoleZUpFrame(RobotSide.LEFT);
               desiredFrame = desiredReferenceFrames.getSoleZUpFrame(RobotSide.LEFT);
            }
         }
         else if (isFootInSupport.get(RobotSide.RIGHT).getValue())
         {
            currentFrame = currentReferenceFrames.getSoleZUpFrame(RobotSide.RIGHT);
            desiredFrame = desiredReferenceFrames.getSoleZUpFrame(RobotSide.RIGHT);
         }
         else
         {
            throw new IllegalArgumentException("We have a flying robot here, such scenario is not handled.");
         }
         RigidBodyTransform rootJointTransform = currentFrame.getTransformToDesiredFrame(desiredFrame);
         // Any yaw-rotation needs to be applied at the desiredFrame's origin. rotationRelocation is used to relocate where the rotation is happening.
         RigidBodyTransform rotationRelocation = desiredFrame.getTransformToDesiredFrame(rootJoint.getFrameAfterJoint());
         rootJointTransform.multiplyInvertOther(rotationRelocation);
         rootJointTransform.preMultiply(rotationRelocation);
         rootJoint.getJointPose().set(rootJointTransform);
         updateTools();
         desiredReferenceFrames.updateFrames();
      }

      // Initialize the initialCenterOfMassPosition and initialFootPoses to match the current state of the robot.
      updateCoMPositionAndFootPoses();

      // By default, always hold the support foot/feet and center of mass in place. This can be changed on the fly by sending a KinematicsToolboxConfigurationMessage.
      holdSupportRigidBodies.set(true);
      enableAutoSupportPolygon.set(true);
      holdCenterOfMassXYPosition.set(true);

      status.setCurrentToolboxState(CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL);
      reportMessage(status);

      return true;
   }

   @Override
   public void updateInternal()
   {
      if (commandInputManager.isNewCommandAvailable(HumanoidKinematicsToolboxConfigurationCommand.class))
      {
         HumanoidKinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(HumanoidKinematicsToolboxConfigurationCommand.class);

         holdCenterOfMassXYPosition.set(command.holdCurrentCenterOfMassXYPosition());
         holdSupportRigidBodies.set(command.holdSupportRigidBodies());
      }

      super.updateInternal();
   }

   /**
    * Sets the {@link #initialCenterOfMassPosition} and {@link #initialFootPoses} to match the current
    * state of {@link #desiredFullRobotModel}.
    */
   private void updateCoMPositionAndFootPoses()
   {
      updateTools();

      initialCenterOfMassPosition.setFromReferenceFrame(centerOfMassFrame);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = desiredFullRobotModel.getFoot(robotSide);
         initialFootPoses.get(robotSide).setFromReferenceFrame(foot.getBodyFixedFrame());
      }
   }

   /**
    * Creates and sets up the feedback control commands for holding the support foot/feet in place. If
    * {@link #holdSupportRigidBodies} is {@code false}, this methods returns {@code null}.
    * <p>
    * Also note that if a user command has been received for a support foot, the command for this foot
    * is not created.
    * </p>
    *
    * @param bufferToPack the buffer used to store the commands for holding the support foot/feet in
    *                     place.
    */
   private void addHoldSupportFootCommands(FeedbackControlCommandBuffer bufferToPack)
   {
      if (!holdSupportRigidBodies.getBooleanValue())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isFootInSupport.get(robotSide).getBooleanValue())
            continue;

         RigidBodyBasics foot = desiredFullRobotModel.getFoot(robotSide);

         // Do not hold the foot position if the user is already controlling it.
         if (isUserControllingRigidBody(foot))
            continue;

         SpatialFeedbackControlCommand feedbackControlCommand = bufferToPack.addSpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, foot);
         feedbackControlCommand.setPrimaryBase(getEndEffectorPrimaryBase(foot));
         feedbackControlCommand.resetControlFrame();
         feedbackControlCommand.resetControlBaseFrame();
         feedbackControlCommand.setGains(getDefaultSpatialGains());
         feedbackControlCommand.setSelectionMatrixToIdentity();
         feedbackControlCommand.setWeightForSolver(supportRigidBodyWeight.getValue());
         feedbackControlCommand.setInverseKinematics(initialFootPoses.get(robotSide), KinematicsToolboxHelper.zeroVector6D);
      }
   }

   /**
    * Creates and sets up the feedback control commands for holding the active contact points in place.
    * 
    * @param bufferToPack
    */
   private void addHoldSupportRigidBodyCommands(FeedbackControlCommandBuffer bufferToPack)
   {
      if (!holdSupportRigidBodies.getBooleanValue())
         return;

      if (contactingRigidBodies.isEmpty())
         return;

      Set<RigidBodyBasics> controlledBodies = new HashSet<RigidBodyBasics>();

      for (int i = 0; i < contactingRigidBodies.size(); i++)
      {
         ContactingRigidBody contactingRigidBody = contactingRigidBodies.get(i);

         // Do not hold the rigid-body position if the user is already controlling it.
         if (isUserControllingRigidBody(contactingRigidBody.rigidBody))
            continue;
         if (!controlledBodies.add(contactingRigidBody.rigidBody))
            continue;

         SpatialFeedbackControlCommand feedbackControlCommand = bufferToPack.addSpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, contactingRigidBody.rigidBody);
         feedbackControlCommand.setPrimaryBase(getEndEffectorPrimaryBase(contactingRigidBody.rigidBody));
         feedbackControlCommand.setControlFrameFixedInEndEffector(contactingRigidBody.contactPointInBodyFixedFrame);
         feedbackControlCommand.resetControlBaseFrame();
         feedbackControlCommand.setGains(getDefaultSpatialGains());
         feedbackControlCommand.getSpatialAccelerationCommand().setSelectionMatrixForLinearControl();
         feedbackControlCommand.setWeightForSolver(supportRigidBodyWeight.getValue());
         feedbackControlCommand.setInverseKinematics(contactingRigidBody.initialPosition, KinematicsToolboxHelper.zeroVector3D);
      }
   }

   /**
    * Creates and sets up the feedback control command for holding the center of mass x and y
    * coordinates in place. If {@link #holdCenterOfMassXYPosition} is {@code false}, this methods
    * returns {@code null}.
    * <p>
    * Also note that if a user command has been received for the center of mass, this methods returns
    * {@code null}.
    * </p>
    *
    * @param bufferToPack the buffer used to store the command for holding the center of mass x and y
    *                     coordinates in place.
    */
   private void addHoldCenterOfMassXYCommand(FeedbackControlCommandBuffer bufferToPack)
   {
      if (!holdCenterOfMassXYPosition.getBooleanValue())
         return;

      // Do not hold the CoM position if the user is already controlling it.
      if (isUserControllingCenterOfMass())
      {
         holdCenterOfMassXYPosition.set(false);
         return;
      }

      centerOfMassPositionToHold.setIncludingFrame(initialCenterOfMassPosition);

      CenterOfMassFeedbackControlCommand feedbackControlCommand = bufferToPack.addCenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(getDefaultSpatialGains().getPositionGains());
      feedbackControlCommand.setWeightForSolver(momentumWeight.getDoubleValue());
      feedbackControlCommand.setSelectionMatrixForLinearXYControl();
      feedbackControlCommand.setInverseKinematics(centerOfMassPositionToHold, KinematicsToolboxHelper.zeroVector3D);
   }

   /**
    * Creates and sets up the {@code JointLimitReductionCommand} from the map
    * {@link #legJointLimitReductionFactors}.
    *
    * @param bufferToPack the buffer used to store the command for reducing the allowed range of motion
    *                     of the leg joints.
    */
   private void addJointLimitReductionCommand(InverseKinematicsCommandBuffer bufferToPack)
   {
      JointLimitReductionCommand jointLimitReductionCommand = bufferToPack.addJointLimitReductionCommand();
      jointLimitReductionCommand.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : desiredFullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJointBasics joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }
   }

   private final RecyclingArrayList<FramePoint3D> activeContactPointPositions = new RecyclingArrayList<>(FramePoint3D::new);

   /**
    * Computes the set of constraints for the momentum x and y components such that the center of mass
    * is guaranteed to remain above the shrunken support polygon.
    *
    * @param bufferToPack the buffer used to store the constraints to submit to the controller core.
    */
   private void addLinearMomentumConvexConstraint2DCommand(InverseKinematicsCommandBuffer bufferToPack)
   {
      if (!enableAutoSupportPolygon.getValue())
         return;
      if (!enableSupportPolygonConstraint.getValue())
         return;
      if (isUserProvidingSupportPolygon())
         return;

      activeContactPointPositions.clear();

      if (hasCapturabilityBasedStatus)
      {
         Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatusInternal.getLeftFootSupportPolygon3d();
         Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatusInternal.getRightFootSupportPolygon3d();
         for (int i = 0; i < leftFootSupportPolygon2d.size(); i++)
            activeContactPointPositions.add().setIncludingFrame(worldFrame, leftFootSupportPolygon2d.get(i));
         for (int i = 0; i < rightFootSupportPolygon2d.size(); i++)
            activeContactPointPositions.add().setIncludingFrame(worldFrame, rightFootSupportPolygon2d.get(i));
      }
      else if (hasMultiContactBalanceStatus)
      {
         Object<Point3D> supportPolygonFromStatus = multiContactBalanceStatusInternal.getSupportPolygon();
         for (int i = 0; i < supportPolygonFromStatus.size(); i++)
            activeContactPointPositions.add().setIncludingFrame(worldFrame, supportPolygonFromStatus.get(i));
      }

      updateSupportPolygonConstraint(activeContactPointPositions, bufferToPack);
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
         capturabilityBasedStatus.getLeftFootSupportPolygon3d().add();
      if (isRightFootInSupport)
         capturabilityBasedStatus.getRightFootSupportPolygon3d().add();
      updateCapturabilityBasedStatus(capturabilityBasedStatus);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      concurrentCapturabilityBasedStatusCopier.getCopyForWriting().set(newStatus);
      concurrentCapturabilityBasedStatusCopier.commit();
   }

   public void updateMultiContactBalanceStatus(MultiContactBalanceStatus newStatus)
   {
      concurrentMultiContactBalanceStatusCopier.getCopyForWriting().set(newStatus);
      concurrentMultiContactBalanceStatusCopier.commit();
   }

   @Override
   protected RigidBodyBasics getEndEffectorPrimaryBase(RigidBodyBasics endEffector)
   {
      return endEffectorToPrimaryBaseMap.get(endEffector);
   }

   @Override
   protected void getAdditionalFeedbackControlCommands(FeedbackControlCommandBuffer bufferToPack)
   {
      addHoldSupportFootCommands(bufferToPack);
      addHoldSupportRigidBodyCommands(bufferToPack);
      addHoldCenterOfMassXYCommand(bufferToPack);
   }

   @Override
   protected void getAdditionalInverseKinematicsCommands(InverseKinematicsCommandBuffer bufferToPack)
   {
      addJointLimitReductionCommand(bufferToPack);
      addLinearMomentumConvexConstraint2DCommand(bufferToPack);
   }

   public YoDouble getMomentumWeight()
   {
      return momentumWeight;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   private static class ContactingRigidBody
   {
      private RigidBodyBasics rigidBody;
      private final FramePoint3D contactPointInBodyFixedFrame = new FramePoint3D();
      private final FramePoint3D initialPosition = new FramePoint3D();

      public ContactingRigidBody()
      {
      }

      public void initialize(RigidBodyBasics rigidBody, ReferenceFrame positionFrame, Point3DReadOnly position)
      {
         this.rigidBody = rigidBody;
         initialPosition.setIncludingFrame(positionFrame, position);
         contactPointInBodyFixedFrame.setIncludingFrame(initialPosition);
         contactPointInBodyFixedFrame.changeFrame(rigidBody.getBodyFixedFrame());
      }
   }
}
