package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.MultiContactBalanceStatus;
import gnu.trove.map.hash.TIntObjectHashMap;
import toolbox_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
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
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Integer;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;

import static toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD;
import static toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL;
import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

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
   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashCodeMap = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<OneDoFJointBasics> jointHashCodeMap = new TIntObjectHashMap<>();

   private final Map<RigidBodyBasics, RigidBodyBasics> endEffectorToPrimaryBaseMap = new HashMap<>();
   private final Map<RigidBodyBasics, GeometricJacobian> rootJacobians = new HashMap<>();

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
   private final HashMap<String, YoDouble> jointLimitReductionFactors = new HashMap<>();
   /**
    * Reference to the most recent data received from the controller relative to the balance control.
    * It is used for identifying which foot is in support and thus which foot should be held in place.
    */
   private final ConcurrentCopier<CapturabilityBasedStatus> concurrentCapturabilityBasedStatusCopier = new ConcurrentCopier<>(CapturabilityBasedStatus::new);
   private boolean hasCapturabilityBasedStatus = false;
   private final CapturabilityBasedStatus capturabilityBasedStatusInternal = new CapturabilityBasedStatus();
   /**
    * Whether to add a JointLimitReductionCommand by calling {{@link #addJointLimitReductionCommand}}.
    */
   private final YoBoolean enableJointLimitReduction = new YoBoolean("enableJointLimitReduction", registry);

   /**
    * Reference to the most recent data received from the controller relative to the balance control.
    * It is used for identifying which rigid-body is used for support and thus which rigid-body should
    * be held in place.
    */
   private final ConcurrentCopier<MultiContactBalanceStatus> concurrentMultiContactBalanceStatusCopier = new ConcurrentCopier<>(MultiContactBalanceStatus::new);
   private boolean hasMultiContactBalanceStatus = false;
   private final MultiContactBalanceStatus multiContactBalanceStatusInternal = new MultiContactBalanceStatus();
   private final ExecutionTimer executionTimer = new ExecutionTimer("ikTotal", registry);

   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel,
                                              double updateDT,
                                              YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoRegistry parentRegistry)
   {
      this(commandInputManager,
           statusOutputManager,
           desiredFullRobotModel,
           createListOfControllableRigidBodies(desiredFullRobotModel),
           updateDT,
           yoGraphicsListRegistry,
           parentRegistry);
   }

   /**
    * Creates a new instance of the IK solver as a controller.
    *
    * @param commandInputManager     the command input manager used to receive commands from the network/direct API.
    * @param statusOutputManager     the status output manager used to send status messages to the network/direct API.
    * @param desiredFullRobotModel   the robot model the solver will be working on and storing the latest solution.
    * @param controllableRigidBodies the list of rigid-bodies that can be controlled by the solver. Mostly used for visualization.
    * @param updateDT                the time step used by the solver.
    * @param yoGraphicsListRegistry  the registry used to store the graphics for visualization.
    * @param parentRegistry          the parent registry for this controller.
    */
   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel,
                                              Collection<? extends RigidBodyBasics> controllableRigidBodies,
                                              double updateDT,
                                              YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoRegistry parentRegistry)
   {
      super(commandInputManager,
            statusOutputManager,
            desiredFullRobotModel.getRootJoint(),
            getAllJointsExcludingHands(desiredFullRobotModel),
            controllableRigidBodies,
            updateDT,
            yoGraphicsListRegistry,
            parentRegistry);

      this.desiredFullRobotModel = desiredFullRobotModel;
      desiredReferenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel, centerOfMassFrame, null);

      desiredFullRobotModel.getElevator().subtreeStream().forEach(rigidBody -> rigidBodyHashCodeMap.put(rigidBody.hashCode(), rigidBody));
      desiredFullRobotModel.getRootBody()
                           .subtreeStream()
                           .forEach(rigidBody -> rootJacobians.put(rigidBody,
                                                                   new GeometricJacobian(desiredFullRobotModel.getElevator(),
                                                                                         rigidBody,
                                                                                         ReferenceFrame.getWorldFrame())));
      Arrays.stream(desiredOneDoFJoints).forEach(joint -> jointHashCodeMap.put(joint.hashCode(), joint));

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

      populateDefaultJointLimitReductionFactors();
   }

   /**
    * Setting up the map holding the joint limit reduction factors and setting to default values.
    * To modify reduction factors on-the-fly use {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private void populateDefaultJointLimitReductionFactors()
   {
      for (int i = 0; i < desiredOneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = desiredOneDoFJoints[i];
         YoDouble limitReductionFactor = new YoDouble(joint.getName() + "LimitReductionFactor", registry);
         jointLimitReductionFactors.put(joint.getName(), limitReductionFactor);
      }

      double defaultHipJointReduction = 0.05;
      for (RobotSide robotSide : RobotSide.values)
      {
         setJointLimitReductionFactor(desiredFullRobotModel.getLegJoint(robotSide, LegJointName.HIP_PITCH).getName(), defaultHipJointReduction);
         setJointLimitReductionFactor(desiredFullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL).getName(), defaultHipJointReduction);
         setJointLimitReductionFactor(desiredFullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW).getName(), defaultHipJointReduction);
      }
   }

   private void setJointLimitReductionFactor(String jointName, double jointLimitReductionFactor)
   {
      if (jointLimitReductionFactors.containsKey(jointName))
         jointLimitReductionFactors.get(jointName).set(jointLimitReductionFactor);
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
         listOfControllableRigidBodies.add(desiredFullRobotModel.getForearm(robotSide));
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
      RobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      FullHumanoidRobotModel robot = robotModel.createFullRobotModel();
      defaultRobotInitialSetup.initializeFullRobotModel(robot);

      for (OneDoFJointBasics joint : getDesiredOneDoFJoints())
      {
         double q_priv = robot.getOneDoFJointByName(joint.getName()).getQ();
         privilegedConfiguration.put(joint, q_priv);
      }

      setInitialRobotConfiguration(privilegedConfiguration);
   }

   public void setCollisionModel(RobotCollisionModel collisionModel)
   {
      if (collisionModel != null)
         registerRobotCollidables(collisionModel.getRobotCollidables(getDesiredFullRobotModel().getElevator()));
   }

   private final RigidBodyTransform rootJointTransform = new RigidBodyTransform();
   private final RigidBodyTransform rotationRelocation = new RigidBodyTransform();
   private final RigidBodyTransform initialTransform = new RigidBodyTransform();
   private final RigidBodyTransform desiredTransform = new RigidBodyTransform();

   @Override
   public boolean initialize()
   {
      firstTick = true;
      KinematicsToolboxOutputStatus status = new KinematicsToolboxOutputStatus();
      status.setJointNameHash(-1);
      status.setSolutionQuality(Double.NaN);

      resetInternalData();

      boolean wasRobotUpdated = desiredRobotStateUpdater.updateRobotConfiguration(rootJoint, desiredOneDoFJoints);
      if (!wasRobotUpdated)
      {
         commandInputManager.clearAllCommands();
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

      contactingRigidBodies.clear();

      if (hasCapturabilityBasedStatus)
      {
         processCapturabilityBasedStatus(capturabilityBasedStatus);
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

         computeSupportZUpTransform(desiredFullRobotModel, initialTransform); // The robot is at the current initial configuration.

         initializePrivilegedConfiguration(); // The robot is now at the privileged configuration.
         rootJoint.getJointPose().setToZero();
         desiredFullRobotModel.updateFrames();
         computeSupportZUpTransform(desiredFullRobotModel, desiredTransform); // The robot is at the privileged configuration.

         rootJointTransform.setAndInvert(desiredTransform);
         rootJointTransform.multiply(initialTransform);
         // Any yaw-rotation needs to be applied at the desiredFrame's origin. rotationRelocation is used to relocate where the rotation is happening.
         rotationRelocation.setAndInvert(rootJoint.getJointPose());
         rotationRelocation.multiply(desiredTransform);

         rootJointTransform.multiplyInvertOther(rotationRelocation);
         rootJointTransform.preMultiply(rotationRelocation);
         rootJoint.getJointPose().set(rootJointTransform);
      }
      else
      {
         initializePrivilegedConfiguration();
      }

      // Initialize the initialCenterOfMassPosition and initialFootPoses to match the current state of the robot.
      updateCoMPositionAndFootPoses();

      // By default, always hold the support foot/feet and center of mass in place. This can be changed on the fly by sending a KinematicsToolboxConfigurationMessage.
      holdSupportRigidBodies.set(true);
      enableAutoSupportPolygon.set(true);
      holdCenterOfMassXYPosition.set(true);
      enableJointLimitReduction.set(true);

      status.setCurrentToolboxState(CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL);
      reportMessage(status);

      return true;
   }

   private void computeSupportZUpTransform(FullHumanoidRobotModel fullRobotModel, RigidBodyTransform transformToPack)
   {

      if (isFootInSupport.get(RobotSide.LEFT).getValue())
      {
         if (isFootInSupport.get(RobotSide.RIGHT).getValue())
         {
            computeMidZUpTransform(fullRobotModel.getSoleFrame(RobotSide.LEFT), fullRobotModel.getSoleFrame(RobotSide.RIGHT), transformToPack);
         }
         else
         {
            computeZUpTransform(fullRobotModel.getSoleFrame(RobotSide.LEFT), transformToPack);
         }
      }
      else if (isFootInSupport.get(RobotSide.RIGHT).getValue())
      {
         computeZUpTransform(fullRobotModel.getSoleFrame(RobotSide.RIGHT), transformToPack);
      }
      else
      {
         throw new IllegalArgumentException("We have a flying robot here, such scenario is not handled.");
      }
   }

   private static void computeMidZUpTransform(ReferenceFrame frameA, ReferenceFrame frameB, RigidBodyTransformBasics transformToPack)
   {
      RigidBodyTransform transformA = frameA.getTransformToRoot();
      RigidBodyTransform transformB = frameB.getTransformToRoot();

      transformToPack.getTranslation().interpolate(transformA.getTranslation(), transformB.getTranslation(), 0.5);
      transformToPack.getRotation().setToYawOrientation(AngleTools.computeAngleAverage(transformA.getRotation().getYaw(), transformB.getRotation().getYaw()));
   }

   private static void computeZUpTransform(ReferenceFrame frame, RigidBodyTransformBasics transformToPack)
   {
      RigidBodyTransform transform = frame.getTransformToRoot();
      transformToPack.getTranslation().set(transform.getTranslation());
      transformToPack.getRotation().setToYawOrientation(transform.getRotation().getYaw());
   }

   @Override
   public void updateInternal()
   {
      executionTimer.startMeasurement();

      if (commandInputManager.isNewCommandAvailable(HumanoidKinematicsToolboxConfigurationCommand.class))
      {
         HumanoidKinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(HumanoidKinematicsToolboxConfigurationCommand.class);

         holdCenterOfMassXYPosition.set(command.holdCurrentCenterOfMassXYPosition());
         holdSupportRigidBodies.set(command.holdSupportRigidBodies());
         enableJointLimitReduction.set(command.enableJointLimitReduction());

         if (command.hasCustomJointRestrictionLimits())
         {
            // Clear joint limit restrictions
            for (int i = 0; i < desiredOneDoFJoints.length; i++)
            {
               jointLimitReductionFactors.get(desiredOneDoFJoints[i].getName()).set(0.0);
            }

            // Update joint limit restrictions
            for (int i = 0; i < command.getNumberOfCustomJointRestrictionLimits(); i++)
            {
               OneDoFJointBasics joint = jointHashCodeMap.get(command.getJointLimitReductionHashCode(i));
               if (joint == null)
                  continue;
               double jointLimitReductionFactor = command.getJointRestrictionLimitFactor(i);
               jointLimitReductionFactors.get(joint.getName()).set(jointLimitReductionFactor);
            }
         }
      }

      MultiContactBalanceStatus multiContactBalanceStatus = concurrentMultiContactBalanceStatusCopier.getCopyForReading();
      hasMultiContactBalanceStatus = multiContactBalanceStatus != null;
      if (hasMultiContactBalanceStatus)
      {
         multiContactBalanceStatusInternal.set(multiContactBalanceStatus);
         processMultiContactBalanceStatus(multiContactBalanceStatusInternal);
      }

      super.updateInternal();

      executionTimer.stopMeasurement();
   }

   @Override
   protected void updateTools()
   {
      // Overriding the default implementation to reduce the number of times the reference frames are updated.
      desiredReferenceFrames.updateFrames();
   }

   /**
    * Sets the {@link #initialCenterOfMassPosition} and {@link #initialFootPoses} to match the current
    * state of {@link #desiredFullRobotModel}.
    */
   protected void updateCoMPositionAndFootPoses()
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

      Set<RigidBodyBasics> controlledBodies = new HashSet<>();

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
    * {@link #jointLimitReductionFactors}.
    *
    * @param bufferToPack the buffer used to store the command for reducing the allowed range of motion
    *                     of the leg joints.
    */
   private void addJointLimitReductionCommand(InverseKinematicsCommandBuffer bufferToPack)
   {
      if (!enableJointLimitReduction.getValue())
         return;

      JointLimitReductionCommand jointLimitReductionCommand = bufferToPack.addJointLimitReductionCommand();
      jointLimitReductionCommand.clear();

      for (int i = 0; i < desiredOneDoFJoints.length; i++)
      {
         double reductionFactor = jointLimitReductionFactors.get(desiredOneDoFJoints[i].getName()).getValue();
         if (reductionFactor <= 0.0 || reductionFactor > 1.0)
            continue;
         jointLimitReductionCommand.addReductionFactor(desiredOneDoFJoints[i], reductionFactor);
      }
   }

   private final RecyclingArrayList<FramePoint3D> activeContactPointPositions = new RecyclingArrayList<>(FramePoint3D::new);

   /**
    * Sets the contact state of the feet and overall support polygon given the robot's active contact points.
    */
   private void processCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      for (RobotSide robotside : RobotSide.values)
         isFootInSupport.get(robotside).set(HumanoidMessageTools.unpackIsSupportFoot(capturabilityBasedStatus, robotside));

      if (!isUserProvidingSupportPolygon())
      {
         Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatus.getLeftFootSupportPolygon3d();
         Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatus.getRightFootSupportPolygon3d();
         for (int i = 0; i < leftFootSupportPolygon2d.size(); i++)
            activeContactPointPositions.add().setIncludingFrame(worldFrame, leftFootSupportPolygon2d.get(i));
         for (int i = 0; i < rightFootSupportPolygon2d.size(); i++)
            activeContactPointPositions.add().setIncludingFrame(worldFrame, rightFootSupportPolygon2d.get(i));

         updateSupportPolygonConstraint(activeContactPointPositions);
      }
   }

   /**
    * Sets the current supporting rigid bodies and sets the support region either from the contact points
    * or by solving for the multi-contact support region.
    * <p>
    * If requested, the supporting rigid bodies are held stationary, see {@link #addHoldSupportRigidBodyCommands}
    */
   private void processMultiContactBalanceStatus(MultiContactBalanceStatus multiContactBalanceStatus)
   {
      for (RobotSide robotside : RobotSide.values)
         isFootInSupport.get(robotside).set(false);

      /* Update supporting rigid bodies */
      contactingRigidBodies.clear();
      Object<Point3D> supportPolygon = multiContactBalanceStatus.getContactPointsInWorld();
      Integer supportRigidBodyIds = multiContactBalanceStatus.getSupportRigidBodyIds();

      if (multiContactBalanceStatus.getContactPointsInWorld().size() < 3)
      {
         shrunkSupportPolygon.clear();
         shrunkSupportPolygonVertices.clear();
         return;
      }

      for (int i = 0; i < supportPolygon.size(); i++)
      {
         ContactingRigidBody contactingRigidBody = contactingRigidBodies.add();
         contactingRigidBody.initialize(rigidBodyHashCodeMap.get(supportRigidBodyIds.get(i)), worldFrame, supportPolygon.get(i));
      }
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
   }

   public YoDouble getMomentumWeight()
   {
      return momentumWeight;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   public CommonHumanoidReferenceFrames getDesiredReferenceFrames()
   {
      return desiredReferenceFrames;
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
