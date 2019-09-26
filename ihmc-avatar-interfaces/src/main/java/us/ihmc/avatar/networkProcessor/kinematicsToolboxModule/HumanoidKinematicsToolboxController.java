package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD;
import static controller_msgs.msg.dds.KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL;
import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import java.util.ArrayList;
import java.util.Collection;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidKinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.HumanoidRobotKinematicsCollisionModel;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.LinearMomentumConvexConstraint2DCommand;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.HumanoidKinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;

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
   /**
    * Indicates whether the center of mass x and y coordinates should be held in place for this run. It
    * is {@code true} by default but can be disabled using the message
    * {@link HumanoidKinematicsToolboxConfigurationMessage}.
    */
   private final YoBoolean holdCenterOfMassXYPosition = new YoBoolean("holdCenterOfMassXYPosition", registry);
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
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   /** The active support polygon updated from the most recent robot configuration. */
   private final ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
   /**
    * The active support polygon shrunk by the distance {@code centerOfMassSafeMargin}. This represents
    * the convex horizontal region that the center of mass is constrained to.
    */
   private final List<Point2D> shrunkSupportPolygonVertices = new ArrayList<>();
   /** Helper used for shrink the support polygon. */
   private final ConvexPolygonScaler convexPolygonScaler = new ConvexPolygonScaler();
   /** Distance to shrink the support polygon for safety purpose. */
   private final YoDouble centerOfMassSafeMargin = new YoDouble("centerOfMassSafeMargin",
                                                                "Describes the minimum distance away from the support polygon's edges.",
                                                                registry);
   /** The total mass of the robot. */
   private final double robotMass;

   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                              double updateDT, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(commandInputManager, statusOutputManager, desiredFullRobotModel, createListOfControllableRigidBodies(desiredFullRobotModel), fullRobotModelFactory,
           updateDT, yoGraphicsListRegistry, parentRegistry);
   }

   public HumanoidKinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                              FullHumanoidRobotModel desiredFullRobotModel, Collection<? extends RigidBodyBasics> controllableRigidBodyies,
                                              FullHumanoidRobotModelFactory fullRobotModelFactory, double updateDT,
                                              YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
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
      momentumWeight.set(1.0e-6);
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

   public void setDefaultPrivilegedConfiguration(DRCRobotModel robotModel)
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

      setDefaultPrivilegedConfiguration(privilegedConfiguration);
   }

   public void setCollisionModel(HumanoidRobotKinematicsCollisionModel collisionModel)
   {
      if (collisionModel != null)
         registerCollidables(collisionModel.getRobotCollidables(getDesiredFullRobotModel()));
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

      if (defaultPrivilegedConfigurationMap != null)
      {
         /*
          * Default initial configuration was provided and is set in the super class. The goal here, is to
          * recompute the pose of the root joint such that our initial configuration has its support feet as
          * close as possible to the current robot support feet. This affects the CoM task.
          */
         RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, currentFullRobotModel.getRootJoint(), currentOneDoFJoints);
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

         RigidBodyBasics foot = desiredFullRobotModel.getFoot(robotSide);

         // Do not hold the foot position if the user is already controlling it.
         if (isUserControllingRigidBody(foot))
            continue;

         FramePose3D poseToHold = new FramePose3D(initialFootPoses.get(robotSide));

         SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, foot);
         feedbackControlCommand.setPrimaryBase(getEndEffectorPrimaryBase(foot));
         feedbackControlCommand.setGains(getDefaultGains());
         feedbackControlCommand.setWeightForSolver(footWeight.getDoubleValue());
         feedbackControlCommand.setInverseKinematics(poseToHold, KinematicsToolboxHelper.zeroVector6D);
         inputs.addCommand(feedbackControlCommand);
      }
      return inputs;
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
      feedbackControlCommand.setInverseKinematics(positionToHold, new FrameVector3D());
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
            OneDoFJointBasics joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }
      return jointLimitReductionCommand;
   }

   /**
    * Computes the set of constraints for the momentum x and y components such that the center of mass
    * is guaranteed to remain above the shrunken support polygon.
    * 
    * @return the constraints to submit to the controller core.
    */
   private InverseKinematicsCommandList createLinearMomentumConvexConstraint2DCommand()
   {
      if (!enableSupportPolygonConstraint.getValue() || latestCapturabilityBasedStatusReference.get() == null)
         return null;

      ConvexPolygon2D newSupportPolygon = new ConvexPolygon2D();
      newSupportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(latestCapturabilityBasedStatusReference.get().getLeftFootSupportPolygon2d()));
      newSupportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(latestCapturabilityBasedStatusReference.get().getRightFootSupportPolygon2d()));
      newSupportPolygon.update();

      if (!newSupportPolygon.epsilonEquals(supportPolygon, 5.0e-3))
      { // Update the polygon only if there is an actual update.
         supportPolygon.set(newSupportPolygon);
         ConvexPolygon2D shrunkConvexPolygon = new ConvexPolygon2D();
         convexPolygonScaler.scaleConvexPolygon(supportPolygon, centerOfMassSafeMargin.getValue(), shrunkConvexPolygon);
         shrunkSupportPolygonVertices.clear();
         shrunkConvexPolygon.getPolygonVerticesView().forEach(vertex -> shrunkSupportPolygonVertices.add(new Point2D(vertex)));

         for (int i = shrunkSupportPolygonVertices.size() - 1; i >= 0; i--)
         { // Filtering vertices that barely expand the polygon.
            Point2DReadOnly vertex = shrunkSupportPolygonVertices.get(i);
            Point2DReadOnly previousVertex = ListWrappingIndexTools.getPrevious(i, shrunkSupportPolygonVertices);
            Point2DReadOnly nextVertex = ListWrappingIndexTools.getNext(i, shrunkSupportPolygonVertices);

            if (EuclidGeometryTools.distanceFromPoint2DToLine2D(vertex, previousVertex, nextVertex) < 1.0e-3)
               shrunkSupportPolygonVertices.remove(i);
         }
      }

      FramePoint2D centerOfMass = new FramePoint2D(centerOfMassFrame);
      centerOfMass.changeFrame(worldFrame);

      InverseKinematicsCommandList commandList = new InverseKinematicsCommandList();
      double distanceThreshold = 0.25 * centerOfMassSafeMargin.getValue();

      for (int i = 0; i < shrunkSupportPolygonVertices.size(); i++)
      { // Only adding constraints that are close to be violated.
         Point2DReadOnly vertex = shrunkSupportPolygonVertices.get(i);
         Point2DReadOnly nextVertex = ListWrappingIndexTools.getNext(i, shrunkSupportPolygonVertices);
         double signedDistanceToEdge = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(centerOfMass, vertex, nextVertex);

         if (signedDistanceToEdge > -distanceThreshold)
         {
            LinearMomentumConvexConstraint2DCommand command = new LinearMomentumConvexConstraint2DCommand();
            Vector2D h0 = command.addLinearMomentumConstraintVertex();
            Vector2D h1 = command.addLinearMomentumConstraintVertex();
            h0.sub(vertex, centerOfMass);
            h1.sub(nextVertex, centerOfMass);
            h0.scale(robotMass / updateDT);
            h1.scale(robotMass / updateDT);
            commandList.addCommand(command);
         }
      }

      return commandList;
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
   protected RigidBodyBasics getEndEffectorPrimaryBase(RigidBodyBasics endEffector)
   {
      return endEffectorToPrimaryBaseMap.get(endEffector);
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
      commands.addCommand(createLinearMomentumConvexConstraint2DCommand());
      return commands;
   }

   public DoubleProvider getCenterOfMassSafeMargin()
   {
      return centerOfMassSafeMargin;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }
}
