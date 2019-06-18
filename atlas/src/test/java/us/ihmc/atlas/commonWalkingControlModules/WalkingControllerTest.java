package us.ihmc.atlas.commonWalkingControlModules;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.jcodec.common.Assert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.MemoryTools;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class WalkingControllerTest
{
   private static final boolean profile = false;
   private static final boolean showSCS = false;
   private static final double totalTime = 20.0;

   private static final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS, RobotTarget.SCS, false)
   {
      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
         {
            @Override
            public boolean allowUpperBodyMotionDuringLocomotion()
            {
               return true;
            };

            @Override
            public double getMaxICPErrorBeforeSingleSupportX()
            {
               return 1.0;
            };

            @Override
            public double getMaxICPErrorBeforeSingleSupportY()
            {
               return 1.0;
            };
         };
      };
   };

   private static final double gravityZ = 9.81;
   private static final double controlDT = robotModel.getControllerDT();
   private static final double velocityDecay = 0.98;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoDouble yoTime = new YoDouble("time", registry);

   private final StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
   private final CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
   private final HighLevelControlManagerFactory managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
   private final List<ContactablePlaneBody> contactableBodies = new ArrayList<>();

   private final SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();

   private SimulationConstructionSet scs;
   private SideDependentList<TestFootSwitch> updatableFootSwitches;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames referenceFrames;
   private PerfectSimulatedOutputWriter writer;
   private OneDoFJointBasics[] oneDoFJoints;

   private WalkingHighLevelHumanoidController walkingController;
   private WholeBodyControllerCore controllerCore;
   private JointDesiredOutputList controllerOutput;

   private static final double maxDriftRate = 0.2;

   @Test
   public void testForGarbage()
   {
      walkingController.initialize();

      // measure multiple ticks
      LogTools.info("Starting to loop.");
      int tickCount = 0;
      while (yoTime.getDoubleValue() < totalTime || profile)
      {
         doSingleTimeUpdate();

         // send messages from time to time
         if (tickCount > 0)
         {
            if (tickCount % (int) (4.0 / controlDT) == 0)
            {
               LogTools.info("Sending Steps");
               sendFootsteps();
            }
            if (tickCount % (int) (4.0 / controlDT) == (int) (2.0 / controlDT))
            {
               LogTools.info("Sending Chest Trajectory");
               sendChestTrajectory();
            }
            if (tickCount % (int) (4.0 / controlDT) == (int) (3.0 / controlDT))
            {
               LogTools.info("Sending Arm Trajectory");
               sendArmTrajectory();
            }
         }

         tickCount++;

         if (showSCS)
         {
            writer.updateRobotConfigurationBasedOnFullRobotModel();
            scs.setTime(yoTime.getDoubleValue());
            scs.tickAndUpdate();
         }
         else
         {
            Tuple3DReadOnly rootPosition = fullRobotModel.getRootJoint().getJointPose().getPosition();
            Point3D min = new Point3D(-0.1, -0.1, 0.5);
            Point3D max = new Point3D(0.1, 0.1, 1.0);
            Vector3D drift = new Vector3D(maxDriftRate, maxDriftRate, maxDriftRate);
            drift.scale(yoTime.getDoubleValue());
            min.sub(drift);
            max.add(drift);
            BoundingBox3D boundingBox = new BoundingBox3D(min, max);
            boolean insideInclusive = boundingBox.isInsideInclusive(rootPosition.getX(), rootPosition.getY(), rootPosition.getZ());
            Assert.assertTrue("Robot drifted away.", insideInclusive);
         }
      }
   }

   private void sendFootsteps()
   {
      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(0.3, 0.2);
      MovingReferenceFrame stanceFrame = referenceFrames.getSoleZUpFrame(RobotSide.LEFT);

      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = referenceFrames.getSoleZUpFrame(robotSide);
         FramePoint3D location = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
         FrameQuaternion orientation = new FrameQuaternion(soleFrame);

         location.changeFrame(stanceFrame);
         location.setZ(0.0);
         location.setX(0.2);

         location.changeFrame(ReferenceFrame.getWorldFrame());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstep);
      }
      commandInputManager.submitMessage(footsteps);
   }

   private void sendChestTrajectory()
   {
      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      Quaternion orientation = new Quaternion();
      orientation.appendYawRotation(Math.toRadians(-10.0));
      orientation.appendRollRotation(Math.toRadians(10.0));
      SO3TrajectoryMessage so3Trajectory = message.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(referenceFrames.getPelvisZUpFrame()));
      SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(0.5);
      trajectoryPoint.getOrientation().set(orientation);
      trajectoryPoint.getAngularVelocity().set(new Vector3D());
      trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.setTime(1.0);
      trajectoryPoint.getOrientation().set(new Quaternion());
      trajectoryPoint.getAngularVelocity().set(new Vector3D());
      commandInputManager.submitMessage(message);
   }

   private void sendArmTrajectory()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);

         for (int jointIdx = 0; jointIdx < joints.length; jointIdx++)
         {
            OneDoFJointBasics joint = joints[jointIdx];
            double angle1 = MathTools.clamp(Math.toRadians(45.0), joint.getJointLimitLower() + 0.05, joint.getJointLimitUpper() - 0.05);
            double angle2 = MathTools.clamp(0.0, joint.getJointLimitLower() + 0.05, joint.getJointLimitUpper() - 0.05);
            OneDoFJointTrajectoryMessage jointTrajectoryMessage = message.getJointspaceTrajectory().getJointTrajectoryMessages().add();
            jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(0.5, angle1, 0.0));
            jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(1.0, angle2, 0.0));
         }
         commandInputManager.submitMessage(message);
      }
   }

   private void doSingleTimeUpdate()
   {
      // (1) compute foot switches based on sole height
      for (RobotSide side : RobotSide.values)
      {
         updatableFootSwitches.get(side).update();
      }

      // (2) do control and compute desired accelerations
      walkingController.doAction();
      ControllerCoreCommand coreCommand = walkingController.getControllerCoreCommand();
      controllerCore.submitControllerCoreCommand(coreCommand);
      controllerCore.compute();

      // (3) integrate accelerations in full robot model
      integrate();

      // update viz and advance time
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      yoTime.add(robotModel.getControllerDT());
   }

   private final Quaternion newOrientation = new Quaternion();
   private final Quaternion orientation = new Quaternion();
   private final Vector3D newAngularVelocity = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D desiredAngularAcceleration = new Vector3D();

   private final Point3D position = new Point3D();
   private final Point3D newPosition = new Point3D();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D newLinearVelocity = new Vector3D();
   private final Vector3D desiredLinearAcceleration = new Vector3D();

   private final FrameVector3D frameLinearVelocity = new FrameVector3D();
   private final FrameVector3D frameAngularVelocity = new FrameVector3D();
   private final Twist rootJointTwist = new Twist();

   private final FramePoint3D solePosition = new FramePoint3D();

   private void integrate()
   {
      // fix one foot to the ground:
      ReferenceFrame fixedFrame = findFrameToFix(footStates, referenceFrames);
      solePosition.setToZero(fixedFrame);
      solePosition.changeFrame(ReferenceFrame.getWorldFrame());
      double zCorrection = 0.0;

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         JointDesiredOutputReadOnly jointDesireds = controllerOutput.getJointDesiredOutput(joint);

         if (jointDesireds.hasDesiredAcceleration())
         {
            double q = joint.getQ();
            double qd = joint.getQd();
            double qdd = jointDesireds.getDesiredAcceleration();

            double qNew = q + controlDT * qd + 0.5 * controlDT * controlDT * qdd;
            double qdNew = qd + controlDT * qdd;

            joint.setQ(qNew);
            joint.setQd(velocityDecay * qdNew);
         }
      }

      RootJointDesiredConfigurationDataReadOnly rootJointOutput = controllerCore.getOutputForRootJoint();
      DenseMatrix64F desiredAcceleration = rootJointOutput.getDesiredAcceleration();
      desiredAngularAcceleration.set(desiredAcceleration.get(0), desiredAcceleration.get(1), desiredAcceleration.get(2));
      desiredLinearAcceleration.set(desiredAcceleration.get(3), desiredAcceleration.get(4), desiredAcceleration.get(5));

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      position.set(rootJoint.getJointPose().getPosition());
      linearVelocity.set(rootJoint.getJointTwist().getLinearPart());

      newPosition.set(desiredLinearAcceleration);
      newPosition.scale(0.5 * controlDT);
      newPosition.add(linearVelocity);
      newPosition.scale(controlDT);
      newPosition.add(position);
      newLinearVelocity.set(desiredLinearAcceleration);
      newLinearVelocity.scale(controlDT);
      newLinearVelocity.add(linearVelocity);

      RotationTools.integrateAngularVelocity(angularVelocity, controlDT, newOrientation);
      newOrientation.preMultiply(orientation);
      newAngularVelocity.set(desiredAngularAcceleration);
      newAngularVelocity.scale(controlDT);
      newAngularVelocity.add(angularVelocity);

      newPosition.addZ(-zCorrection);

      rootJoint.setJointOrientation(newOrientation);
      rootJoint.setJointPosition(newPosition);
      rootJoint.updateFramesRecursively();
      frameLinearVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), newLinearVelocity);
      frameAngularVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), newAngularVelocity);
      frameLinearVelocity.scale(velocityDecay);
      frameAngularVelocity.scale(velocityDecay);
      frameLinearVelocity.changeFrame(rootJoint.getFrameAfterJoint());
      frameAngularVelocity.changeFrame(rootJoint.getFrameAfterJoint());
      rootJointTwist.setIncludingFrame(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(), frameAngularVelocity,
                         frameLinearVelocity);
      rootJoint.setJointTwist(rootJointTwist);
   }

   private ReferenceFrame findFrameToFix(SideDependentList<YoEnum<ConstraintType>> footStates, HumanoidReferenceFrames referenceFrames)
   {
      int sidesInDoubleSupport = 0;
      RobotSide sideToFix = null;
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footStates.get(robotSide).getEnumValue() == ConstraintType.FULL)
         {
            sidesInDoubleSupport++;
            sideToFix = robotSide;
         }
      }

      if (sidesInDoubleSupport == 2)
      {
         return referenceFrames.getMidFeetZUpFrame();
      }
      else if (sidesInDoubleSupport == 1)
      {
         return referenceFrames.getSoleZUpFrame(sideToFix);
      }

      throw new RuntimeException("One foot needs to be in full support at all times for this test.");
   }

   private void createControllerCore()
   {
      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      JointBasics[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, jointsToOptimizeFor, centerOfMassFrame,
                                                                            momentumOptimizationSettings, yoGraphicsListRegistry, registry);
      toolbox.setupForInverseDynamicsSolver(contactableBodies);

      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);

      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      controllerOutput = new JointDesiredOutputList(fullRobotModel.getControllableOneDoFJoints());
      controllerCore = new WholeBodyControllerCore(toolbox, template, controllerOutput, registry);
   }

   private void createWalkingControllerAndSetUpManagerFactory()
   {
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      double omega0 = walkingControllerParameters.getOmega0();

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(), contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionalContactNames.get(i), additionalContactTransforms.get(i));
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      for (RobotSide robotSide : RobotSide.values)
         contactableBodies.add(feet.get(robotSide));
      contactableBodies.addAll(additionalContacts);
      contactableBodiesFactory.disposeFactory();

      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityZ;
      updatableFootSwitches = TestFootSwitch.createFootSwitches(feet, totalRobotWeight, referenceFrames.getSoleZUpFrames());
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>(updatableFootSwitches);

      HighLevelHumanoidControllerToolbox controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, null, yoTime,
                                                                                                    gravityZ, omega0, feet, controlDT, null, contactableBodies,
                                                                                                    yoGraphicsListRegistry);
      registry.addChild(controllerToolbox.getYoVariableRegistry());

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultInitialTransferTime,
                                                                              defaultFinalTransferTime, feet, statusOutputManager, yoTime,
                                                                              yoGraphicsListRegistry, registry);
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);

      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 controllerToolbox);
   }

   @SuppressWarnings("unchecked")
   @BeforeEach
   public void setupTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      oneDoFJoints = fullRobotModel.getOneDoFJoints();

      setupRobotAndCopyConfiguration(robot);
      createWalkingControllerAndSetUpManagerFactory();
      createControllerCore();

      registry.addChild(walkingController.getYoVariableRegistry());
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      for (RobotSide robotSide : RobotSide.values)
      {
         String name = robotSide.getLowerCaseName() + "FootAssumeCopOnEdge";
         YoBoolean variable = (YoBoolean) registry.getVariable(name);
         variable.set(true);

         name = robotSide.getLowerCaseName() + "FootAssumeFootBarelyLoaded";
         variable = (YoBoolean) registry.getVariable(name);
         variable.set(true);

         name = robotSide.getCamelCaseNameForStartOfExpression() + "FootCurrentState";
         YoEnum<ConstraintType> footState = (YoEnum<ConstraintType>) registry.getVariable(name);
         footStates.put(robotSide, footState);
      }

      ParameterLoaderHelper.loadParameters(this, robotModel.getWholeBodyControllerParametersFile(), registry);

      if (showSCS)
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setCreateGUI(true);
         scs = new SimulationConstructionSet(robot);
         scs.setDT(robotModel.getControllerDT(), 1);
         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         plotterFactory.createOverheadPlotter();
         scs.setCameraTracking(true, true, true, true);
         scs.addYoVariableRegistry(registry);
         scs.setGroundVisible(false);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setTime(0.0);
         scs.tickAndUpdate();
      }
   }

   private void setupRobotAndCopyConfiguration(HumanoidFloatingRootJointRobot robot)
   {
      robot.setDynamic(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      initialSetup.initializeRobot(robot, robotModel.getJointMap());

      writer = new PerfectSimulatedOutputWriter(robot, fullRobotModel);

      for (OneDoFJointBasics revoluteJoint : fullRobotModel.getOneDoFJoints())
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);
         revoluteJoint.setQ(oneDoFJoint.getQ());
         oneDoFJoint.setQd(0.0);
         revoluteJoint.setQd(0.0);
      }

      FloatingJoint floatingJoint = robot.getRootJoint();
      FloatingJointBasics sixDoFJoint = fullRobotModel.getRootJoint();
      RigidBodyTransform transform = new RigidBodyTransform();
      floatingJoint.getTransformToWorld(transform);
      sixDoFJoint.setJointConfiguration(transform);

      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
   }

   @AfterEach
   public void tearDown()
   {
      if (showSCS)
      {
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();
         scs.play();
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
