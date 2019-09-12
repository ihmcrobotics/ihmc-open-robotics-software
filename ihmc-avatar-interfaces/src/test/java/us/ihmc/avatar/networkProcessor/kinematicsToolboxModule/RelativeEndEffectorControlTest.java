package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTestRobots.UpperBodyWithTwoManipulators;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis;
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
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePose3D;

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
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;

   private Robot robot;
   private RigidBodyBasics rootBody;
   private Pair<FloatingJointBasics, OneDoFJointBasics[]> desiredFullRobotModel;

   private SimulationConstructionSet scs;

   public void setup(RobotDescription robotDescription)
   {
      if (mainRegistry == null)
         mainRegistry = new YoVariableRegistry("main");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      if (desiredFullRobotModel == null)
         desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
      if (rootBody == null)
         rootBody = MultiBodySystemTools.getRootBody(desiredFullRobotModel.getRight()[0].getPredecessor());

      robot = new RobotFromDescription(robotDescription);
      robot.setDynamic(false);
      robot.setGravity(0);
      robot.addYoVariableRegistry(mainRegistry);

      if (visualize)
      {
         scs = new SimulationConstructionSet(robot, simulationTestingParameters);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
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

      FeedbackControlCommandList commandTemplate = rootBody.subtreeStream().map(RelativeEndEffectorControlTest::toFeedbackControlCommand)
                                                           .collect(FeedbackControlCommandList::new,
                                                                    FeedbackControlCommandList::addCommand,
                                                                    FeedbackControlCommandList::addCommandList);
      controllerCore = new WholeBodyControllerCore(controllerCoreToolbox, commandTemplate, mainRegistry);
      controllerCore.compute();
   }

   private static SpatialFeedbackControlCommand toFeedbackControlCommand(RigidBodyBasics endEffector)
   {
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      command.set(MultiBodySystemTools.getRootBody(endEffector), endEffector);
      return command;
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (mainRegistry != null)
      {
         mainRegistry.closeAndDispose();
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
         scs.closeAndDispose();
         scs = null;
      }
   }

   @Test
   public void testControlEndEffectorRelativeToOtherEndEffector()
   {
      mainRegistry = new YoVariableRegistry("main");
      UpperBodyWithTwoManipulators robotDescription = new UpperBodyWithTwoManipulators();
      desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
      rootBody = MultiBodySystemTools.getRootBody(desiredFullRobotModel.getRight()[0].getPredecessor());

      RigidBodyBasics lHand = rootBody.subtreeStream().filter(body -> body.getName().equals("leftHandLink")).findFirst().get();
      RigidBodyBasics rHand = rootBody.subtreeStream().filter(body -> body.getName().equals("rightHandLink")).findFirst().get();

      MovingReferenceFrame lHandFrame = lHand.getBodyFixedFrame();
      MovingReferenceFrame rHandFrame = rHand.getBodyFixedFrame();
      YoFixedFrameTwist leftRelativeTwist = new YoFixedFrameTwist("leftRelative", lHandFrame, rHandFrame, lHandFrame, mainRegistry);
      YoFixedFrameTwist rightRelativeTwist = new YoFixedFrameTwist("rightRelative", rHandFrame, lHandFrame, rHandFrame, mainRegistry);
      YoFramePose3D leftRelativePose = new YoFramePose3D("leftRelativePose", rHandFrame, mainRegistry);
      YoFramePose3D rightRelativePose = new YoFramePose3D("rightRelativePose", rHandFrame, mainRegistry);

      setup(robotDescription);
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
      JointAnglesWriter writer = new JointAnglesWriter(robot, desiredFullRobotModel.getLeft(), desiredFullRobotModel.getRight());

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
         writer.updateRobotConfigurationBasedOnFullRobotModel();
         if (visualize)
            scs.tickAndUpdate();
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
         writer.updateRobotConfigurationBasedOnFullRobotModel();

         Twist twist = new Twist();
         rHandFrame.getTwistRelativeToOther(lHandFrame, twist);
         rightRelativeTwist.set(twist);
         lHandFrame.getTwistRelativeToOther(rHandFrame, twist);
         leftRelativeTwist.set(twist);
         leftRelativePose.setFromReferenceFrame(rHandFrame);
         rightRelativePose.setFromReferenceFrame(lHandFrame);
         if (visualize)
            scs.tickAndUpdate();
         assertTrue(leftRelativeTwist.getAngularPart().length() < 0.01);
         assertTrue(leftRelativeTwist.getLinearPart().length() < 0.01);
      }
   }

   @Test
   public void testInequalityEndEffector()
   {
      mainRegistry = new YoVariableRegistry("main");
      UpperBodyWithTwoManipulators robotDescription = new UpperBodyWithTwoManipulators();
      desiredFullRobotModel = KinematicsToolboxControllerTestRobots.createInverseDynamicsRobot(robotDescription);
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
         LinkGraphicsDescription handGraphics = robotDescription.getLinkDescription(robotSide.getCamelCaseName() + "WristYaw").getLinkGraphics();
         handGraphics.identity();
         handGraphics.translate(0.0, 0.0, 0.15);
         handGraphics.addSphere(0.01, YoAppearance.Orange());
      }

      setup(robotDescription);
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
      JointAnglesWriter writer = new JointAnglesWriter(robot, desiredFullRobotModel.getLeft(), desiredFullRobotModel.getRight());

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
         writer.updateRobotConfigurationBasedOnFullRobotModel();
         if (visualize)
            scs.tickAndUpdate();
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
         writer.updateRobotConfigurationBasedOnFullRobotModel();

         Twist twist = new Twist();
         rHandFrame.getTwistRelativeToOther(lHandFrame, twist);
         rightRelativeTwist.set(twist);
         lHandFrame.getTwistRelativeToOther(rHandFrame, twist);
         leftRelativeTwist.set(twist);
         leftRelativePose.setFromReferenceFrame(rHandFrame);
         rightRelativePose.setFromReferenceFrame(lHandFrame);
         if (visualize)
            scs.tickAndUpdate();

         assertTrue(yDistance.getValue() > -1.0e-3);
      }
   }

   private static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circlePositionAt(time, frequency, radius, center, new Vector3D());
   }

   private static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
   {
      double angle = 2.0 * Math.PI * frequency * time;
      Vector3D offset = new Vector3D(Axis.Z);
      offset.scale(radius);
      RotationMatrixTools.applyRollRotation(angle, offset, offset);
      FramePoint3D position = new FramePoint3D();
      position.add(center, offset);
      position.scaleAdd(time, centerVelocity, position);
      return position;
   }

   private static FrameVector3D circleLinearVelocityAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circleLinearVelocityAt(time, frequency, radius, center, new Vector3D());
   }

   private static FrameVector3D circleLinearVelocityAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
   {
      double omega = 2.0 * Math.PI * frequency;
      double angle = omega * time;
      Vector3D offset = new Vector3D(Axis.Z);
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
