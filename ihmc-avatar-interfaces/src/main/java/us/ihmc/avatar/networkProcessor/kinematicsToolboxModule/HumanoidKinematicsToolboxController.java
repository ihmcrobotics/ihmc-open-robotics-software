package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL;
import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import java.util.*;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.LinearMomentumConvexConstraint2DCommand;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
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

   private final Map<RigidBodyBasics, RigidBodyBasics> endEffectorToPrimaryBaseMap = new HashMap<>();

   /**
    * Updated during the initialization phase, this set of two {@link YoBoolean}s is used to know which
    * foot is currently used for support in the walking controller.
    */
   private final SideDependentList<YoBoolean> isFootInSupport = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the poses of the feet are stored so they
    * can be held in place during the optimization process such that the solution will be statically
    * reachable.
    */
   private final SideDependentList<YoFramePose3D> initialFootPoses = new SideDependentList<>();
   /**
    * Updated during the initialization phase, this is where the robot's center of mass position is
    * stored so it can be held in place during the optimization process such that the solution will be
    * statically reachable.
    */
   private final YoFramePoint3D initialCenterOfMassPosition = new YoFramePoint3D("initialCenterOfMass", worldFrame, registry);

   /**
    * Indicates whether the support foot/feet should be held in place for this run. It is {@code true}
    * by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdSupportFootPose = new YoBoolean("holdSupportFootPose", registry);
   private final FramePose3D footPoseToHold = new FramePose3D();
   /**
    * Indicates whether the center of mass x and y coordinates should be held in place for this run. It
    * is {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdCenterOfMassXYPosition = new YoBoolean("holdCenterOfMassXYPosition", registry);
   private final FramePoint3D centerOfMassPositionToHold = new FramePoint3D();
   /**
    * Indicates whether the projection of the center of mass is to be contained inside the support
    * polygon.It is {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean enableSupportPolygonConstraint = new YoBoolean("enableSupportPolygonConstraint", registry);
   /**
    * Default weight used when holding the support foot/feet in place. It is rather high such that they
    * do not deviate much from their initial poses.
    */
   private final YoDouble footWeight = new YoDouble("footWeight", registry);
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

   /** The active support polygon updated from the most recent robot configuration. */
   private final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
   /**
    * The active support polygon shrunk by the distance {@code centerOfMassSafeMargin}. This represents
    * the convex horizontal region that the center of mass is constrained to.
    */
   private final RecyclingArrayList<Point2D> shrunkSupportPolygonVertices = new RecyclingArrayList<>(Point2D.class);
   /** Helper used for shrink the support polygon. */
   private final ConvexPolygonScaler convexPolygonScaler = new ConvexPolygonScaler();
   private final ConvexPolygon2D newSupportPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D shrunkConvexPolygon = new ConvexPolygon2D();
   private final FramePoint2D centerOfMass = new FramePoint2D();
   /** Distance to shrink the support polygon for safety purpose. */
   private final YoDouble centerOfMassSafeMargin = new YoDouble("centerOfMassSafeMargin",
                                                                "Describes the minimum distance away from the support polygon's edges.",
                                                                registry);
   /** The total mass of the robot. */
   private final double robotMass;

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
      this.currentFullRobotModel = fullRobotModelFactory.createFullRobotModel();
      currentOneDoFJoints = getAllJointsExcludingHands(currentFullRobotModel);
      currentReferenceFrames = new HumanoidReferenceFrames(currentFullRobotModel);

      robotMass = TotalMassCalculator.computeSubTreeMass(desiredFullRobotModel.getElevator());

      footWeight.set(200.0);
      momentumWeight.set(0.001);
      centerOfMassSafeMargin.set(0.04); // Same as the walking controller.

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
         if (desiredFullRobotModel.getHand(robotSide) != null)
            listOfControllableRigidBodies.add(desiredFullRobotModel.getHand(robotSide));
         listOfControllableRigidBodies.add(desiredFullRobotModel.getFoot(robotSide));
      }

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

      // Using the most recent CapturabilityBasedStatus received from the walking controller to figure out which foot is in support.
      CapturabilityBasedStatus capturabilityBasedStatus = concurrentCapturabilityBasedStatusCopier.getCopyForReading();
      hasCapturabilityBasedStatus = capturabilityBasedStatus != null;
      if (hasCapturabilityBasedStatus)
         capturabilityBasedStatusInternal.set(capturabilityBasedStatus);

      if (hasCapturabilityBasedStatus)
      {
         for (RobotSide robotside : RobotSide.values)
            isFootInSupport.get(robotside).set(HumanoidMessageTools.unpackIsSupportFoot(capturabilityBasedStatusInternal, robotside));
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
            isFootInSupport.get(robotSide).set(true);
      }

      if (initialRobotConfigurationMap != null)
      {
         /*
          * Default initial configuration was provided and is set in the super class. The goal here, is to
          * recompute the pose of the root joint such that our initial configuration has its support feet as
          * close as possible to the current robot support feet. This affects the CoM task.
          */
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationDataInternal, currentFullRobotModel.getRootJoint(), currentOneDoFJoints);
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
      enableSupportPolygonConstraint.set(true);
      holdSupportFootPose.set(true);
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

         enableSupportPolygonConstraint.set(command.enableSupportPolygonConstraint());
         holdCenterOfMassXYPosition.set(command.holdCurrentCenterOfMassXYPosition());
         holdSupportFootPose.set(command.holdSupportFootPositions());
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
    * {@link #holdSupportFootPose} is {@code false}, this methods returns {@code null}.
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
      if (!holdSupportFootPose.getBooleanValue())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isFootInSupport.get(robotSide).getBooleanValue())
            continue;

         RigidBodyBasics foot = desiredFullRobotModel.getFoot(robotSide);

         // Do not hold the foot position if the user is already controlling it.
         if (isUserControllingRigidBody(foot))
            continue;

         footPoseToHold.setIncludingFrame(initialFootPoses.get(robotSide));

         SpatialFeedbackControlCommand feedbackControlCommand = bufferToPack.addSpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, foot);
         feedbackControlCommand.setPrimaryBase(getEndEffectorPrimaryBase(foot));
         feedbackControlCommand.resetControlFrame();
         feedbackControlCommand.resetControlBaseFrame();
         feedbackControlCommand.setGains(getDefaultSpatialGains());
         feedbackControlCommand.setSelectionMatrixToIdentity();
         feedbackControlCommand.setWeightForSolver(footWeight.getDoubleValue());
         feedbackControlCommand.setInverseKinematics(footPoseToHold, KinematicsToolboxHelper.zeroVector6D);
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

   /**
    * Computes the set of constraints for the momentum x and y components such that the center of mass
    * is guaranteed to remain above the shrunken support polygon.
    * 
    * @param bufferToPack the buffer used to store the constraints to submit to the controller core.
    */
   private void addLinearMomentumConvexConstraint2DCommand(InverseKinematicsCommandBuffer bufferToPack)
   {
      if (!enableSupportPolygonConstraint.getValue() || !hasCapturabilityBasedStatus)
         return;

      newSupportPolygon.clear();
      Object<Point3D> leftFootSupportPolygon2d = capturabilityBasedStatusInternal.getLeftFootSupportPolygon2d();
      Object<Point3D> rightFootSupportPolygon2d = capturabilityBasedStatusInternal.getRightFootSupportPolygon2d();
      for (int i = 0; i < leftFootSupportPolygon2d.size(); i++)
         newSupportPolygon.addVertex(leftFootSupportPolygon2d.get(i));
      for (int i = 0; i < rightFootSupportPolygon2d.size(); i++)
         newSupportPolygon.addVertex(rightFootSupportPolygon2d.get(i));
      newSupportPolygon.update();

      if (!newSupportPolygon.epsilonEquals(supportPolygon, 5.0e-3))
      { // Update the polygon only if there is an actual update.
         supportPolygon.set(newSupportPolygon);
         convexPolygonScaler.scaleConvexPolygon(supportPolygon, centerOfMassSafeMargin.getValue(), shrunkConvexPolygon);
         shrunkSupportPolygonVertices.clear();
         for (int i = 0; i < shrunkConvexPolygon.getNumberOfVertices(); i++)
            shrunkSupportPolygonVertices.add().set(shrunkConvexPolygon.getVertex(i));

         for (int i = shrunkSupportPolygonVertices.size() - 1; i >= 0; i--)
         { // Filtering vertices that barely expand the polygon.
            Point2DReadOnly vertex = shrunkSupportPolygonVertices.get(i);
            Point2DReadOnly previousVertex = ListWrappingIndexTools.getPrevious(i, shrunkSupportPolygonVertices);
            Point2DReadOnly nextVertex = ListWrappingIndexTools.getNext(i, shrunkSupportPolygonVertices);

            if (EuclidGeometryTools.distanceFromPoint2DToLine2D(vertex, previousVertex, nextVertex) < 1.0e-3)
               shrunkSupportPolygonVertices.remove(i);
         }
      }

      centerOfMass.setToZero(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      double distanceThreshold = 0.25 * centerOfMassSafeMargin.getValue();

      for (int i = 0; i < shrunkSupportPolygonVertices.size(); i++)
      { // Only adding constraints that are close to be violated.
         Point2DReadOnly vertex = shrunkSupportPolygonVertices.get(i);
         Point2DReadOnly nextVertex = ListWrappingIndexTools.getNext(i, shrunkSupportPolygonVertices);
         double signedDistanceToEdge = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(centerOfMass, vertex, nextVertex);

         if (signedDistanceToEdge > -distanceThreshold)
         {
            LinearMomentumConvexConstraint2DCommand command = bufferToPack.addLinearMomentumConvexConstraint2DCommand();
            command.clear();
            Vector2D h0 = command.addLinearMomentumConstraintVertex();
            Vector2D h1 = command.addLinearMomentumConstraintVertex();
            h0.sub(vertex, centerOfMass);
            h1.sub(nextVertex, centerOfMass);
            h0.scale(robotMass / updateDT);
            h1.scale(robotMass / updateDT);
         }
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
         capturabilityBasedStatus.getLeftFootSupportPolygon2d().add();
      if (isRightFootInSupport)
         capturabilityBasedStatus.getRightFootSupportPolygon2d().add();
      updateCapturabilityBasedStatus(capturabilityBasedStatus);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      concurrentCapturabilityBasedStatusCopier.getCopyForWriting().set(newStatus);
      concurrentCapturabilityBasedStatusCopier.commit();
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

   public YoDouble getCenterOfMassSafeMargin()
   {
      return centerOfMassSafeMargin;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }
}
