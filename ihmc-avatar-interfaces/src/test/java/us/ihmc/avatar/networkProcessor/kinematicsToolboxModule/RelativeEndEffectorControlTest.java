package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobots.UpperBodyWithTwoManipulators;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * The goal here is to validate control approach used in the {@code KinematicsToolboxModule} when
 * doing collision avoidance.
 */
public class RelativeEndEffectorControlTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   private static final double controlDT = 1.0e-4;
   static
   {
      simulationTestingParameters.setKeepSCSUp(true);
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private YoRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;

   private Robot robot;
   private RigidBodyBasics rootBody;
   private Pair<FloatingJointBasics, OneDoFJointBasics[]> desiredFullRobotModel;

   private SimulationConstructionSet2 scs;

   public void setup(RobotDefinition robotDefinition)
   {
      if (mainRegistry == null)
         mainRegistry = new YoRegistry("main");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      if (desiredFullRobotModel == null)
         desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDefinition);
      if (rootBody == null)
         rootBody = MultiBodySystemTools.getRootBody(desiredFullRobotModel.getRight()[0].getPredecessor());

      robot = new Robot(robotDefinition, SimulationConstructionSet2.inertialFrame);
      robot.getRegistry().addChild(mainRegistry);

      if (visualize)
      {
         scs = new SimulationConstructionSet2(SimulationConstructionSet2.doNothingPhysicsEngine());
         scs.addRobot(robot);
         scs.addYoGraphics(SCS1GraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
         scs.start(true, true, true);
         scs.setCameraFocusPosition(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
      }
   }

   private KinematicsToolboxOptimizationSettings optimizationSettings;
   private ReferenceFrame centerOfMassFrame = null;
   private WholeBodyControlCoreToolbox controllerCoreToolbox = null;
   private WholeBodyControllerCore controllerCore = null;

   public void createControllerCore()
   {
      optimizationSettings = new KinematicsToolboxOptimizationSettings();
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", worldFrame, rootBody);
      controllerCoreToolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                              0.0,
                                                              desiredFullRobotModel.getKey(),
                                                              desiredFullRobotModel.getRight(),
                                                              centerOfMassFrame,
                                                              optimizationSettings,
                                                              yoGraphicsListRegistry,
                                                              mainRegistry);
      controllerCoreToolbox.setupForInverseKinematicsSolver();
      controllerCoreToolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());

      FeedbackControllerTemplate template = new FeedbackControllerTemplate();
      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
      {
         template.enableSpatialFeedbackController(rigidBody);
      }
      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, template, mainRegistry);
      controllerCore.compute();
   }

   @AfterEach
   public void tearDown()
   {
      if (visualize && simulationTestingParameters.getKeepSCSUp())
      {
         scs.pause();
         scs.waitUntilVisualizerDown();
      }

      if (mainRegistry != null)
      {
         mainRegistry.clear();
         mainRegistry = null;
      }

      yoGraphicsListRegistry = null;
      desiredFullRobotModel = null;
      robot = null;
      optimizationSettings = null;
      centerOfMassFrame = null;
      controllerCoreToolbox = null;
      controllerCore = null;

      if (scs != null)
      {
         scs.shutdownSession();
         scs = null;
      }
   }

   @Test
   public void testControlEndEffectorRelativeToOtherEndEffector()
   {
      mainRegistry = new YoRegistry("main");
      UpperBodyWithTwoManipulators robotDefinition = new UpperBodyWithTwoManipulators();
      desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDefinition);
      rootBody = MultiBodySystemTools.getRootBody(desiredFullRobotModel.getRight()[0].getPredecessor());

      RigidBodyBasics lHand = MultiBodySystemTools.findRigidBody(rootBody, "leftHandLink");
      RigidBodyBasics rHand = MultiBodySystemTools.findRigidBody(rootBody, "rightHandLink");

      MovingReferenceFrame lHandFrame = lHand.getBodyFixedFrame();
      MovingReferenceFrame rHandFrame = rHand.getBodyFixedFrame();
      YoFixedFrameTwist leftRelativeTwist = new YoFixedFrameTwist("leftRelative", lHandFrame, rHandFrame, lHandFrame, mainRegistry);
      YoFixedFrameTwist rightRelativeTwist = new YoFixedFrameTwist("rightRelative", rHandFrame, lHandFrame, rHandFrame, mainRegistry);
      YoFramePose3D leftRelativePose = new YoFramePose3D("leftRelativePose", rHandFrame, mainRegistry);
      YoFramePose3D rightRelativePose = new YoFramePose3D("rightRelativePose", rHandFrame, mainRegistry);

      setup(robotDefinition);
      createControllerCore();

      // Set to the same values as in the KinematicsToolboxController
      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setPositionMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);
      gains.setOrientationProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setOrientationMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

      double circleRadius = 0.15;
      double circleFrequency = 1.0;
      double circleOmega = 2.0 * Math.PI * circleFrequency;
      Point3D leftHandCircleCenter = new Point3D(0.4, 0.3, 0.4);
      YawPitchRoll leftHandOrientation = new YawPitchRoll(0.0, 0., Math.PI / 2.0);

      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_MID_RANGE);
      List<? extends JointBasics> allDesiredJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());

      FramePoint3D leftInitialPosition = new FramePoint3D(worldFrame);
      leftInitialPosition.set(leftHandCircleCenter);
      leftInitialPosition.addZ(circleRadius);
      FramePoint3D rightInitialPosition = new FramePoint3D(worldFrame);
      rightInitialPosition.set(leftHandCircleCenter);
      rightInitialPosition.setY(-rightInitialPosition.getY());
      rightInitialPosition.addZ(circleRadius);

      for (double t = 0.0; t < 0.2; t += controlDT)
      {
         SpatialFeedbackControlCommand leftHandCommand = new SpatialFeedbackControlCommand();
         leftHandCommand.set(rootBody, lHand);
         leftHandCommand.setGains(gains);
         leftHandCommand.setWeightForSolver(10.0);
         leftHandCommand.setInverseKinematics(leftInitialPosition, new FrameVector3D());
         leftHandCommand.setInverseKinematics(new FrameQuaternion(worldFrame, leftHandOrientation), new FrameVector3D());

         SpatialFeedbackControlCommand rightHandCommand = new SpatialFeedbackControlCommand();
         rightHandCommand.set(rootBody, rHand);
         rightHandCommand.setGains(gains);
         rightHandCommand.setWeightForSolver(10.0);
         rightHandCommand.setInverseKinematics(rightInitialPosition, new FrameVector3D());
         rightHandCommand.setInverseKinematics(new FrameQuaternion(worldFrame, leftHandOrientation), new FrameVector3D());

         ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
         controllerCoreCommand.addFeedbackControlCommand(leftHandCommand);
         controllerCoreCommand.addFeedbackControlCommand(rightHandCommand);
         controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommand);
         controllerCore.submitControllerCoreCommand(controllerCoreCommand);
         controllerCore.compute();
         KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(),
                                                                       desiredFullRobotModel.getLeft(),
                                                                       desiredFullRobotModel.getRight());
         MultiBodySystemTools.copyJointsState(allDesiredJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);

         if (visualize)
            scs.simulateNow(1);
      }

      for (double t = 0.0; t < 2.0; t += controlDT)
      {
         FramePoint3D desiredPosition = circlePositionAt(t, circleFrequency, circleRadius, leftHandCircleCenter);
         FrameVector3D desiredVelocity = circleLinearVelocityAt(t, circleFrequency, circleRadius, leftHandCircleCenter);
         YawPitchRoll desiredOrientation = new YawPitchRoll(leftHandOrientation);
         desiredOrientation.setYaw(Math.sin(circleOmega * t) * 0.17);

         SpatialFeedbackControlCommand leftHandCommand = new SpatialFeedbackControlCommand();
         leftHandCommand.set(rootBody, lHand);
         leftHandCommand.setGains(gains);
         leftHandCommand.setWeightForSolver(10.0);
         leftHandCommand.setInverseKinematics(desiredPosition, desiredVelocity);
         leftHandCommand.setInverseKinematics(new FrameQuaternion(worldFrame, desiredOrientation), new FrameVector3D());

         SpatialVelocityCommand rightHandCommand = new SpatialVelocityCommand();
         rightHandCommand.set(lHand, rHand);
         rightHandCommand.setLinearVelocity(rHandFrame, new FrameVector3D(rHandFrame));
         rightHandCommand.setWeight(10.0);
         rightHandCommand.setConstraintType(ConstraintType.EQUALITY);

         ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
         controllerCoreCommand.addFeedbackControlCommand(leftHandCommand);
         controllerCoreCommand.addInverseKinematicsCommand(rightHandCommand);
         controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommand);
         controllerCore.submitControllerCoreCommand(controllerCoreCommand);
         controllerCore.compute();
         KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(),
                                                                       desiredFullRobotModel.getLeft(),
                                                                       desiredFullRobotModel.getRight());
         MultiBodySystemTools.copyJointsState(allDesiredJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);

         Twist twist = new Twist();
         rHandFrame.getTwistRelativeToOther(lHandFrame, twist);
         rightRelativeTwist.set(twist);
         lHandFrame.getTwistRelativeToOther(rHandFrame, twist);
         leftRelativeTwist.set(twist);
         leftRelativePose.setFromReferenceFrame(rHandFrame);
         rightRelativePose.setFromReferenceFrame(lHandFrame);
         if (visualize)
            scs.simulateNow(1);
         assertTrue(leftRelativeTwist.getAngularPart().length() < 0.01);
         assertTrue(leftRelativeTwist.getLinearPart().length() < 0.01);
      }
   }

   @Test
   public void testInequalityEndEffector()
   {
      mainRegistry = new YoRegistry("main");
      UpperBodyWithTwoManipulators robotDefinition = new UpperBodyWithTwoManipulators();
      desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDefinition);
      rootBody = MultiBodySystemTools.getRootBody(desiredFullRobotModel.getRight()[0].getPredecessor());

      RigidBodyBasics lHand = rootBody.subtreeStream().filter(body -> body.getName().equals("leftHandLink")).findFirst().get();
      RigidBodyBasics rHand = rootBody.subtreeStream().filter(body -> body.getName().equals("rightHandLink")).findFirst().get();
      SideDependentList<RigidBodyBasics> hands = new SideDependentList<>(lHand, rHand);

      MovingReferenceFrame lHandFrame = lHand.getBodyFixedFrame();
      MovingReferenceFrame rHandFrame = rHand.getBodyFixedFrame();
      YoFixedFrameTwist leftRelativeTwist = new YoFixedFrameTwist("leftRelative", lHandFrame, rHandFrame, lHandFrame, mainRegistry);
      YoFixedFrameTwist rightRelativeTwist = new YoFixedFrameTwist("rightRelative", rHandFrame, lHandFrame, rHandFrame, mainRegistry);
      YoFramePose3D leftRelativePose = new YoFramePose3D("leftRelativePose", rHandFrame, mainRegistry);
      YoFramePose3D rightRelativePose = new YoFramePose3D("rightRelativePose", rHandFrame, mainRegistry);

      SideDependentList<FramePoint3DReadOnly> handTips = new SideDependentList<>(side -> new FramePoint3D(hands.get(side).getParentJoint().getFrameAfterJoint(),
                                                                                                          0.0,
                                                                                                          0.0,
                                                                                                          0.15));
      SideDependentList<YoFramePoint3D> yoHandTips = new SideDependentList<>(side -> new YoFramePoint3D(side.getCamelCaseName() + "HandTip",
                                                                                                        worldFrame,
                                                                                                        mainRegistry));
      YoDouble yDistance = new YoDouble("yDistance", mainRegistry);
      YoDouble yDotMax = new YoDouble("yDotMax", mainRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         List<VisualDefinition> handGraphics = robotDefinition.getRigidBodyDefinition(robotSide.getCamelCaseName() + "HandLink").getVisualDefinitions();
         handGraphics.add(new VisualDefinition(new Point3D(0, 0, 0.2), new Sphere3DDefinition(0.01), new MaterialDefinition(ColorDefinitions.Orange())));
      }

      setup(robotDefinition);
      if (visualize)
         scs.stopSimulationThread();
      createControllerCore();

      // Set to the same values as in the KinematicsToolboxController
      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setPositionMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);
      gains.setOrientationProportionalGains(1200.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      gains.setOrientationMaxFeedbackAndFeedbackRate(1500.0, Double.POSITIVE_INFINITY);

      double circleRadius = 0.15;
      double circleFrequency = 5.0;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.4, side.negateIfRightSide(0.25), 0.4));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ? new Vector3D(0.0, -0.1, 0.0)
            : new Vector3D());
      SideDependentList<YawPitchRoll> handOrientations = new SideDependentList<>(side -> new YawPitchRoll(0.0, 0.0, side.negateIfRightSide(Math.PI / 2.0)));

      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_MID_RANGE);
      List<? extends JointBasics> allDesiredJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());

      for (double t = 0.0; t < 0.2; t += controlDT)
      {
         ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);

         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint3D initialPosition = new FramePoint3D(worldFrame, circleCenters.get(robotSide));
            initialPosition.addZ(circleRadius);
            SpatialFeedbackControlCommand handCommand = new SpatialFeedbackControlCommand();
            handCommand.set(rootBody, hands.get(robotSide));
            handCommand.setGains(gains);
            handCommand.setWeightForSolver(10.0);
            handCommand.setInverseKinematics(initialPosition, new FrameVector3D());
            handCommand.setInverseKinematics(new FrameQuaternion(worldFrame, handOrientations.get(robotSide)), new FrameVector3D());
            controllerCoreCommand.addFeedbackControlCommand(handCommand);
         }

         controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommand);
         controllerCore.submitControllerCoreCommand(controllerCoreCommand);
         controllerCore.compute();
         KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(),
                                                                       desiredFullRobotModel.getLeft(),
                                                                       desiredFullRobotModel.getRight());

         MultiBodySystemTools.copyJointsState(allDesiredJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);

         if (visualize)
            scs.simulateNow(1);
      }

      for (double t = 0.0; t < 2.0; t += controlDT)
      {
         ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);

         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint3D desiredPosition = circlePositionAt(t,
                                                            robotSide.negateIfRightSide(circleFrequency),
                                                            circleRadius,
                                                            circleCenters.get(robotSide),
                                                            circleCenterVelocities.get(robotSide));
            FrameVector3D desiredVelocity = circleLinearVelocityAt(t,
                                                                   robotSide.negateIfRightSide(circleFrequency),
                                                                   circleRadius,
                                                                   circleCenters.get(robotSide),
                                                                   circleCenterVelocities.get(robotSide));

            SpatialFeedbackControlCommand handCommand = new SpatialFeedbackControlCommand();
            handCommand.set(rootBody, hands.get(robotSide));
            handCommand.setGains(gains);
            handCommand.setWeightForSolver(10.0);
            handCommand.setInverseKinematics(desiredPosition, desiredVelocity);
            handCommand.setInverseKinematics(new FrameQuaternion(worldFrame, handOrientations.get(robotSide)), new FrameVector3D());
            controllerCoreCommand.addFeedbackControlCommand(handCommand);
         }

         for (RobotSide robotSide : RobotSide.values)
            yoHandTips.get(robotSide).setMatchingFrame(handTips.get(robotSide));

         yDistance.set(yoHandTips.get(RobotSide.LEFT).getY() - yoHandTips.get(RobotSide.RIGHT).getY());
         yDotMax.set(yDistance.getValue() / controlDT);

         SpatialVelocityCommand rightHandConstraint = new SpatialVelocityCommand();
         rightHandConstraint.set(lHand, rHand);
         rightHandConstraint.getDesiredLinearVelocity().setY(yDotMax.getValue());
         rightHandConstraint.getSelectionMatrix().clearSelection();
         rightHandConstraint.getSelectionMatrix().selectLinearY(true);
         rightHandConstraint.getSelectionMatrix().setSelectionFrame(worldFrame);
         rightHandConstraint.setConstraintType(ConstraintType.LEQ_INEQUALITY);
         controllerCoreCommand.addInverseKinematicsCommand(rightHandConstraint);

         controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommand);
         controllerCore.submitControllerCoreCommand(controllerCoreCommand);
         controllerCore.compute();
         KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(),
                                                                       desiredFullRobotModel.getLeft(),
                                                                       desiredFullRobotModel.getRight());
         MultiBodySystemTools.copyJointsState(allDesiredJoints, robot.getAllJoints(), JointStateType.CONFIGURATION);

         Twist twist = new Twist();
         rHandFrame.getTwistRelativeToOther(lHandFrame, twist);
         rightRelativeTwist.set(twist);
         lHandFrame.getTwistRelativeToOther(rHandFrame, twist);
         leftRelativeTwist.set(twist);
         leftRelativePose.setFromReferenceFrame(rHandFrame);
         rightRelativePose.setFromReferenceFrame(lHandFrame);
         if (visualize)
            scs.simulateNow(1);

         assertTrue(yDistance.getValue() > -1.0e-3);
      }
   }

   public static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circlePositionAt(time, frequency, radius, center, new Vector3D());
   }

   public static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
   {
      double angle = 2.0 * Math.PI * frequency * time;
      Vector3D offset = new Vector3D(Axis3D.Z);
      offset.scale(radius);
      RotationMatrixTools.applyRollRotation(angle, offset, offset);
      FramePoint3D position = new FramePoint3D();
      position.add(center, offset);
      position.scaleAdd(time, centerVelocity, position);
      return position;
   }

   public static FrameVector3D circleLinearVelocityAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circleLinearVelocityAt(time, frequency, radius, center, new Vector3D());
   }

   public static FrameVector3D circleLinearVelocityAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
   {
      double omega = 2.0 * Math.PI * frequency;
      double angle = omega * time;
      Vector3D offset = new Vector3D(Axis3D.Z);
      offset.scale(radius);
      RotationMatrixTools.applyRollRotation(angle, offset, offset);
      FrameVector3D linearVelocity = new FrameVector3D();
      RotationMatrixTools.applyRollRotation(Math.PI, offset, offset);
      offset.scale(omega);
      linearVelocity.set(offset);
      linearVelocity.add(centerVelocity);
      return linearVelocity;
   }
}
