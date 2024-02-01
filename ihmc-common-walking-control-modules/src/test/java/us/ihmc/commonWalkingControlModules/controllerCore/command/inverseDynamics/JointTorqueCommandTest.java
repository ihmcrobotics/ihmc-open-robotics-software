package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointTorqueCommandTest
{
   @Test
   public void testJointTorqueObjectiveOnFixedBaseMechanism()
   {
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(name);

      double linkLength = 1.0;

      // only penalize joint acceleration at unit cost
      double jointAccelerationWeight = 1.0;

      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      RevoluteJoint joint1 = new RevoluteJoint("joint1", elevator, Axis3D.X);
      RigidBody link1 = new RigidBody("link1", joint1, 1.0, 0.2, 0.2, 1.0, new Point3D(0.0, 0.0, -0.5 * linkLength));
      RevoluteJoint joint2 = new RevoluteJoint("joint2", link1, new RigidBodyTransform(new Quaternion(), new Point3D(0.0, 0.0, -linkLength)), Axis3D.X);
      RigidBody link2 = new RigidBody("link2", joint2, 1.0, 0.2, 0.2, 1.0, new Point3D(0.0, 0.0, -0.5 * linkLength));
      OneDoFJointBasics[] controlledJoints = new OneDoFJointBasics[] {joint1, joint2};

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", link2.getBodyFixedFrame());

      SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
      spatialAccelerationCommand.getSelectionMatrix().setLinearAxisSelection(false, true, false);
      spatialAccelerationCommand.getSelectionMatrix().setAngularAxisSelection(false, false, false);
      spatialAccelerationCommand.setAsHardConstraint();
      spatialAccelerationCommand.getDesiredLinearAcceleration().setY(1.0);

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(0.01,
                                                                                       9.81,
                                                                                       null,
                                                                                       controlledJoints,
                                                                                       centerOfMassFrame,
                                                                                       new JointTorqueTestOptimizationSettings(jointAccelerationWeight, 0),
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(controlledJoints);

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      spatialAccelerationCommand.set(elevator, link2);

      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(controlCoreToolbox,
                                                                           new FeedbackControllerTemplate(allPossibleCommands),
                                                                           lowLevelControllerCoreOutput,
                                                                           registry);

      elevator.updateFramesRecursively();
      centerOfMassFrame.update();

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.setLinearAxisSelection(false, true, false);
      selectionMatrix.setAngularAxisSelection(false, false, false);
      DefaultPID3DGains positionGains = new DefaultPID3DGains();
      positionGains.setProportionalGains(0.0, 1.0, 0.0);
      positionGains.setDerivativeGains(0.0, 1.0, 0.0);

      FramePose3D controlFramePose = new FramePose3D(link2.getBodyFixedFrame());
      controlFramePose.setZ(-linkLength / 2.0);

      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
      jacobianCalculator.setKinematicChain(elevator, link2);
      jacobianCalculator.setJacobianFrame(controlFrame);

      /* Test with random desired torques */

      int numberOfTests = 100;
      Random random = new Random(23890);
      for (int i = 0; i < numberOfTests; i++)
      {
         ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

         double q1 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double q2 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double qd1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double qd2 = EuclidCoreRandomTools.nextDouble(random, 1.0);

         joint1.setQ(q1);
         joint2.setQ(q2);
         joint1.setQd(qd1);
         joint2.setQd(qd2);
         elevator.updateFramesRecursively();
         controlFrame.setPoseAndUpdate(controlFramePose);
         spatialAccelerationCommand.getControlFramePose().setIncludingFrame(controlFramePose);
         jacobianCalculator.reset();

         double desiredTorque = EuclidCoreRandomTools.nextDouble(random, 5.0);
         OneDoFJointBasics joint = random.nextBoolean() ? joint1 : joint2;

         controllerCoreCommand.clear();

         JointTorqueCommand jointTorqueCommand = new JointTorqueCommand();
         double jointTorqueWeight = Double.POSITIVE_INFINITY;
         jointTorqueCommand.addJoint(joint, desiredTorque, jointTorqueWeight);
         controllerCoreCommand.addInverseDynamicsCommand(jointTorqueCommand);
         controllerCoreCommand.addInverseDynamicsCommand(spatialAccelerationCommand);

         controllerCore.initialize();
         controllerCore.compute(controllerCoreCommand);
         ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
         JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

         // Assert joint torque matches requested
         Assertions.assertTrue(EuclidCoreTools.epsilonEquals(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(joint), desiredTorque, 1e-12));

         // Desired spatial linear acceleration Y is achievable, check that it's achieved
         DMatrixRMaj spatialAcceleration = computeSpatialAcceleration(jacobianCalculator,
                                                                      lowLevelOneDoFJointDesiredDataHolder,
                                                                      controllerCore.getOutputForRootJoint());
         Assertions.assertTrue(EuclidCoreTools.epsilonEquals(spatialAcceleration.get(4, 0), 1.0, 1e-12));
      }
   }

   @Test
   public void testJointTorqueObjectiveOnFloatingBaseMechanism()
   {
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(name);

      // Simple mechanism that has a point-foot and two vertical prismatic joints. The CoM is placed directly above the foot
      double jointOffset = 1.0;
      double linkMass = 1.0;
      Axis3D jointAxes = Axis3D.Z;

      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint floatingJoint = new SixDoFJoint("floatingJoint", elevator);
      RigidBody upperLink = new RigidBody("upperLink", floatingJoint, 0.1, 0.1, 0.1, linkMass, new Point3D());
      PrismaticJoint upperJoint = new PrismaticJoint("upperJoint", upperLink, new Point3D(0.0, 0.0, -0.5 * jointOffset), jointAxes);
      RigidBody middleLink = new RigidBody("middleLink", upperJoint, 0.1, 0.1, 0.1, linkMass, new Point3D(0.0, 0.0, -0.5 * jointOffset));
      PrismaticJoint lowerJoint = new PrismaticJoint("lowerJoint", middleLink, new Point3D(0.0, 0.0, -0.5 * jointOffset), jointAxes);
      RigidBody contactingLink = new RigidBody("contactingLink", lowerJoint, 0.1, 0.1, 0.1, linkMass, new Point3D(0.0, 0.0, -0.5 * jointOffset));
      floatingJoint.getJointPose().setZ(2.0);
      elevator.updateFramesRecursively();

      // Acceleration/Rho regularization
      double jointAccelerationRegularizationWeight = 0.1;
      double rhoRegularizationWeight = 0.1;

      // Setup single contact point
      List<ContactablePlaneBody> contactableBodies = new ArrayList<>();
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.0, 0.0));
      contactableBodies.add(new ListOfPointsContactablePlaneBody(contactingLink, contactingLink.getBodyFixedFrame(), contactPoints));

      // Setup whole body controller core
      JointBasics[] controlledJoints = new JointBasics[] {floatingJoint, upperJoint, lowerJoint};
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(0.01,
                                                                                       9.81,
                                                                                       floatingJoint,
                                                                                       controlledJoints,
                                                                                       centerOfMassFrame,
                                                                                       new JointTorqueTestOptimizationSettings(jointAccelerationRegularizationWeight,
                                                                                                                               1),
                                                                                       new YoGraphicsListRegistry(),
                                                                                       registry);
      controlCoreToolbox.setupForInverseDynamicsSolver(contactableBodies);
      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(new OneDoFJointReadOnly[] {upperJoint});
      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(controlCoreToolbox,
                                                                           new FeedbackControllerTemplate(allPossibleCommands),
                                                                           lowLevelControllerCoreOutput,
                                                                           registry);
      ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      // Contact state command
      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
      planeContactStateCommand.setContactingRigidBody(contactingLink);
      planeContactStateCommand.setCoefficientOfFriction(1.0);
      planeContactStateCommand.setContactNormal(new FrameVector3D(contactingLink.getBodyFixedFrame(), 0.0, 0.0, 1.0));
      planeContactStateCommand.setHasContactStateChanged(true);
      planeContactStateCommand.addPointInContact(new FramePoint3D(contactingLink.getBodyFixedFrame(), 0.0, 0.0, 0.0));
      planeContactStateCommand.setRhoWeight(0, rhoRegularizationWeight);

      // Spatial acceleration commands - hard constraints
      double stationaryWeight = Double.POSITIVE_INFINITY;
      SpatialAccelerationCommand contactingLinkAccelerationCommand = new SpatialAccelerationCommand();
      contactingLinkAccelerationCommand.set(elevator, contactingLink);
      contactingLinkAccelerationCommand.getControlFramePose().setToZero(contactingLink.getBodyFixedFrame());
      contactingLinkAccelerationCommand.getDesiredLinearAcceleration().setToZero();
      contactingLinkAccelerationCommand.setWeight(stationaryWeight);
      contactingLinkAccelerationCommand.getSelectionMatrix().getAngularPart().setAxisSelection(false, false, false);
      contactingLinkAccelerationCommand.getSelectionMatrix().getLinearPart().setAxisSelection(false, false, true);

      SpatialAccelerationCommand floatingLinkAccelerationCommand = new SpatialAccelerationCommand();
      floatingLinkAccelerationCommand.set(elevator, upperLink);
      floatingLinkAccelerationCommand.getDesiredLinearAcceleration().setToZero();
      floatingLinkAccelerationCommand.getSelectionMatrix().getAngularPart().setAxisSelection(false, false, false);
      floatingLinkAccelerationCommand.getSelectionMatrix().getLinearPart().setAxisSelection(false, false, true);
      floatingLinkAccelerationCommand.getControlFramePose().setToZero(upperLink.getBodyFixedFrame());
      floatingLinkAccelerationCommand.setWeight(stationaryWeight);

      int numTests = 100;
      Random random = new Random(23890);

      for (int i = 0; i < numTests; i++)
      {
         // Set to random joint position/velocities
         double q1 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double q2 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double qd1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double qd2 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         upperJoint.setQ(q1);
         lowerJoint.setQ(q2);
         upperJoint.setQd(qd1);
         lowerJoint.setQd(qd2);
         elevator.updateFramesRecursively();

         // Joint torque command - request specific downward force on one of the joints
         double desiredTorqueWeight = Double.POSITIVE_INFINITY;
         JointTorqueCommand jointTorqueCommand = new JointTorqueCommand();
         double desiredActuatorForce = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         OneDoFJointBasics requestedJoint = lowerJoint;
         jointTorqueCommand.addJoint(requestedJoint, desiredActuatorForce, desiredTorqueWeight);

         // Setup jacobians for validating controller output
         GeometricJacobianCalculator contactingLinkJacobian = new GeometricJacobianCalculator();
         contactingLinkJacobian.setKinematicChain(elevator, contactingLink);
         contactingLinkJacobian.setJacobianFrame(contactingLink.getBodyFixedFrame());

         GeometricJacobianCalculator upperLinkJacobian = new GeometricJacobianCalculator();
         upperLinkJacobian.setKinematicChain(elevator, upperLink);
         upperLinkJacobian.setJacobianFrame(upperLink.getBodyFixedFrame());

         // Run controller core twice - contact state, dynamic matrices and joint torque task aren't processed in order of dependency
         controllerCore.initialize();

         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
         controllerCoreCommand.addInverseDynamicsCommand(contactingLinkAccelerationCommand);
         controllerCoreCommand.addInverseDynamicsCommand(floatingLinkAccelerationCommand);
         controllerCore.compute(controllerCoreCommand);

         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
         controllerCoreCommand.addInverseDynamicsCommand(contactingLinkAccelerationCommand);
         controllerCoreCommand.addInverseDynamicsCommand(floatingLinkAccelerationCommand);
         controllerCoreCommand.addInverseDynamicsCommand(jointTorqueCommand);
         controllerCore.compute(controllerCoreCommand);

         JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCore.getOutputForLowLevelController();
         RootJointDesiredConfigurationDataReadOnly rootJointDesiredData = controllerCore.getOutputForRootJoint();

         // Check that achieved spatial accelerations are zero
         DMatrixRMaj contactingLinkAcceleration = computeSpatialAcceleration(contactingLinkJacobian,
                                                                             lowLevelOneDoFJointDesiredDataHolder,
                                                                             rootJointDesiredData);
         DMatrixRMaj upperLinkAcceleration = computeSpatialAcceleration(upperLinkJacobian, lowLevelOneDoFJointDesiredDataHolder, rootJointDesiredData);
         Assertions.assertTrue(EuclidCoreTools.epsilonEquals(contactingLinkAcceleration.get(5, 0), 0.0, 1e-9));
         Assertions.assertTrue(EuclidCoreTools.epsilonEquals(upperLinkAcceleration.get(5, 0), 0.0, 1e-9));

         // Check that achieved torque matches requested
         Assertions.assertTrue(EuclidCoreTools.epsilonEquals(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(requestedJoint),
                                                             desiredActuatorForce,
                                                             1e-9));
      }
   }

   @Test
   public void testJointTorqueConstraintOnFixedBaseMechanism()
   {
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(name);

      double linkLength = 1.0;

      // only penalize joint acceleration at unit cost
      double jointAccelerationWeight = 1.0;

      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      RevoluteJoint joint1 = new RevoluteJoint("joint1", elevator, Axis3D.X);
      RigidBody link1 = new RigidBody("link1", joint1, 1.0, 0.2, 0.2, 1.0, new Point3D(0.0, 0.0, -0.5 * linkLength));
      RevoluteJoint joint2 = new RevoluteJoint("joint2", link1, new RigidBodyTransform(new Quaternion(), new Point3D(0.0, 0.0, -linkLength)), Axis3D.X);
      RigidBody link2 = new RigidBody("link2", joint2, 1.0, 0.2, 0.2, 1.0, new Point3D(0.0, 0.0, -0.5 * linkLength));
      OneDoFJointBasics[] controlledJoints = new OneDoFJointBasics[] {joint1, joint2};

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", link2.getBodyFixedFrame());

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(0.01,
                                                                                       9.81,
                                                                                       null,
                                                                                       controlledJoints,
                                                                                       centerOfMassFrame,
                                                                                       new JointTorqueTestOptimizationSettings(jointAccelerationWeight, 0),
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(controlledJoints);

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();

      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(controlCoreToolbox,
                                                                           new FeedbackControllerTemplate(allPossibleCommands),
                                                                           lowLevelControllerCoreOutput,
                                                                           registry);

      elevator.updateFramesRecursively();
      centerOfMassFrame.update();

      FramePose3D controlFramePose = new FramePose3D(link2.getBodyFixedFrame());
      controlFramePose.setZ(-linkLength / 2.0);

      GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
      jacobianCalculator.setKinematicChain(elevator, link2);
      jacobianCalculator.setJacobianFrame(controlFrame);

      /* Test with random desired torques */

      int numberOfTests = 100;
      Random random = new Random(23890);
      for (int i = 0; i < numberOfTests; i++)
      {
         ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

         double q1 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double q2 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double qd1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double qd2 = EuclidCoreRandomTools.nextDouble(random, 1.0);

         joint1.setQ(q1);
         joint2.setQ(q2);
         joint1.setQd(qd1);
         joint2.setQd(qd2);
         elevator.updateFramesRecursively();
         controlFrame.setPoseAndUpdate(controlFramePose);
         jacobianCalculator.reset();

         double desiredTorque = EuclidCoreRandomTools.nextDouble(random, 5.0);
         double torqueLimit = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         OneDoFJointBasics joint = random.nextBoolean() ? joint1 : joint2;

         controllerCoreCommand.clear();

         // Put an upper and lower bound on the allowable torque in the QP
         JointTorqueCommand jointTorqueConstraintUpper = new JointTorqueCommand();
         jointTorqueConstraintUpper.addJoint(joint, torqueLimit);
         jointTorqueConstraintUpper.setConstraintType(ConstraintType.LEQ_INEQUALITY);

         JointTorqueCommand jointTorqueConstraintLower = new JointTorqueCommand();
         jointTorqueConstraintLower.addJoint(joint, -1.0 * torqueLimit);
         jointTorqueConstraintLower.setConstraintType(ConstraintType.GEQ_INEQUALITY);

         // Command a torque (QP should fail if weight is set to infinity)
         JointTorqueCommand jointTorqueCommand = new JointTorqueCommand();
         // double jointTorqueWeight = Double.POSITIVE_INFINITY;
         double jointTorqueWeight = 1000000;
         jointTorqueCommand.addJoint(joint, desiredTorque, jointTorqueWeight);

         controllerCoreCommand.addInverseDynamicsCommand(jointTorqueCommand);
         controllerCoreCommand.addInverseDynamicsCommand(jointTorqueConstraintLower);
         controllerCoreCommand.addInverseDynamicsCommand(jointTorqueConstraintUpper);

         controllerCore.initialize();
         controllerCore.compute(controllerCoreCommand);
         ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
         JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

         double desiredTorqueFromQP = lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(joint);

         // Assert joint torque is less than torque limit
         double epsilon = 1e-8; // epsilon 1e-12 is too restrictive, epsilon 1e-8 is really close, depends on jointTorqueWeight
         if (desiredTorqueFromQP >= torqueLimit + epsilon)
         {
            System.out.println("Iter " + i + ". Upper bound violated. \t QP desired: " + desiredTorqueFromQP + ", \tTorque constraint: " + torqueLimit
                               + ", \tTorque desired: " + desiredTorque);
         }
         if (desiredTorqueFromQP <= -1.0 * torqueLimit - epsilon)
         {
            System.out.println("Iter " + i + ". Lower bound violated. \t QP desired: " + desiredTorqueFromQP + ", \tTorque constraint: " + -1.0 * torqueLimit
                               + ", \tTorque desired: " + desiredTorque);
         }
         Assertions.assertTrue(desiredTorqueFromQP <= torqueLimit + epsilon);
         Assertions.assertTrue(desiredTorqueFromQP >= -1.0 * torqueLimit - epsilon);

         double epsilon2 = 1e-5;
         if (desiredTorque <= torqueLimit + epsilon && desiredTorque >= -torqueLimit - epsilon)
         {
            // Assert joint torque matches requested if it did not exceed the torqueLimit
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(desiredTorqueFromQP, desiredTorque, epsilon2));
         }
         else if (desiredTorque > torqueLimit + epsilon)
         {
            // Assert joint torque is at torqueLimit if desiredTorque exceeded the upper bound
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(desiredTorqueFromQP, torqueLimit, epsilon2));
         }
         else if (desiredTorque < -torqueLimit - epsilon)
         {
            // Assert joint torque is at torqueLimit if desiredTorque subceeded the lower bound
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(desiredTorqueFromQP, -torqueLimit, epsilon2));
         }
         else
         {
            System.out.println("Unhandled case: How in the world did you end up here?");
         }
      }
   }

   @Test
   public void testJointTorqueConstraintOnFloatingBaseMechanism()
   {
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(name);

      // Simple mechanism that has a point-foot and two vertical prismatic joints. The CoM is placed directly above the foot
      double jointOffset = 1.0;
      double linkMass = 1.0;
      Axis3D jointAxes = Axis3D.Z;

      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint floatingJoint = new SixDoFJoint("floatingJoint", elevator);
      RigidBody upperLink = new RigidBody("upperLink", floatingJoint, 0.1, 0.1, 0.1, linkMass, new Point3D());
      PrismaticJoint upperJoint = new PrismaticJoint("upperJoint", upperLink, new Point3D(0.0, 0.0, -0.5 * jointOffset), jointAxes);
      RigidBody middleLink = new RigidBody("middleLink", upperJoint, 0.1, 0.1, 0.1, linkMass, new Point3D(0.0, 0.0, -0.5 * jointOffset));
      PrismaticJoint lowerJoint = new PrismaticJoint("lowerJoint", middleLink, new Point3D(0.0, 0.0, -0.5 * jointOffset), jointAxes);
      RigidBody contactingLink = new RigidBody("contactingLink", lowerJoint, 0.1, 0.1, 0.1, linkMass, new Point3D(0.0, 0.0, -0.5 * jointOffset));
      floatingJoint.getJointPose().setZ(2.0);
      elevator.updateFramesRecursively();

      // Acceleration/Rho regularization
      double jointAccelerationRegularizationWeight = 0.1;
      double rhoRegularizationWeight = 0.1;

      // Setup single contact point
      List<ContactablePlaneBody> contactableBodies = new ArrayList<>();
      List<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(0.0, 0.0));
      contactableBodies.add(new ListOfPointsContactablePlaneBody(contactingLink, contactingLink.getBodyFixedFrame(), contactPoints));

      // Setup whole body controller core
      JointBasics[] controlledJoints = new JointBasics[] {floatingJoint, upperJoint, lowerJoint};
      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(0.01,
                                                                                       9.81,
                                                                                       floatingJoint,
                                                                                       controlledJoints,
                                                                                       centerOfMassFrame,
                                                                                       new JointTorqueTestOptimizationSettings(jointAccelerationRegularizationWeight,
                                                                                                                               1),
                                                                                       new YoGraphicsListRegistry(),
                                                                                       registry);
      controlCoreToolbox.setupForInverseDynamicsSolver(contactableBodies);
      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(new OneDoFJointReadOnly[] {upperJoint});
      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(controlCoreToolbox,
                                                                           new FeedbackControllerTemplate(allPossibleCommands),
                                                                           lowLevelControllerCoreOutput,
                                                                           registry);
      ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      // Contact state command
      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
      planeContactStateCommand.setContactingRigidBody(contactingLink);
      planeContactStateCommand.setCoefficientOfFriction(1.0);
      planeContactStateCommand.setContactNormal(new FrameVector3D(contactingLink.getBodyFixedFrame(), 0.0, 0.0, 1.0));
      planeContactStateCommand.setHasContactStateChanged(true);
      planeContactStateCommand.addPointInContact(new FramePoint3D(contactingLink.getBodyFixedFrame(), 0.0, 0.0, 0.0));
      planeContactStateCommand.setRhoWeight(0, rhoRegularizationWeight);

      // Spatial acceleration commands - hard constraints
      double stationaryWeight = Double.POSITIVE_INFINITY;
      SpatialAccelerationCommand contactingLinkAccelerationCommand = new SpatialAccelerationCommand();
      contactingLinkAccelerationCommand.set(elevator, contactingLink);
      contactingLinkAccelerationCommand.getControlFramePose().setToZero(contactingLink.getBodyFixedFrame());
      contactingLinkAccelerationCommand.getDesiredLinearAcceleration().setToZero();
      contactingLinkAccelerationCommand.setWeight(stationaryWeight);
      contactingLinkAccelerationCommand.getSelectionMatrix().getAngularPart().setAxisSelection(false, false, false);
      contactingLinkAccelerationCommand.getSelectionMatrix().getLinearPart().setAxisSelection(false, false, true);

      SpatialAccelerationCommand floatingLinkAccelerationCommand = new SpatialAccelerationCommand();
      floatingLinkAccelerationCommand.set(elevator, upperLink);
      floatingLinkAccelerationCommand.getDesiredLinearAcceleration().setToZero();
      floatingLinkAccelerationCommand.getSelectionMatrix().getAngularPart().setAxisSelection(false, false, false);
      floatingLinkAccelerationCommand.getSelectionMatrix().getLinearPart().setAxisSelection(false, false, true);
      floatingLinkAccelerationCommand.getControlFramePose().setToZero(upperLink.getBodyFixedFrame());
      floatingLinkAccelerationCommand.setWeight(stationaryWeight);

      int numTests = 100;
      Random random = new Random(23890);

      for (int i = 0; i < numTests; i++)
      {
         // Set to random joint position/velocities
         double q1 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double q2 = EuclidCoreRandomTools.nextDouble(random, 0.3);
         double qd1 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double qd2 = EuclidCoreRandomTools.nextDouble(random, 1.0);
         upperJoint.setQ(q1);
         lowerJoint.setQ(q2);
         upperJoint.setQd(qd1);
         lowerJoint.setQd(qd2);
         elevator.updateFramesRecursively();

         // Joint force command - request specific downward force on one of the joints
         double desiredTorqueWeight = 1000000; // increasing this will require an increase in epsilon
         JointTorqueCommand jointForceCommand = new JointTorqueCommand();
         double desiredActuatorForce = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         OneDoFJointBasics requestedJoint = lowerJoint;
         jointForceCommand.addJoint(requestedJoint, desiredActuatorForce, desiredTorqueWeight);

         // Joint force constraints - Put an upper and lower bound on the allowable force in the QP
         double forceLimit = EuclidCoreRandomTools.nextDouble(random, 1.0, 20.0);
         JointTorqueCommand jointForceConstraintUpper = new JointTorqueCommand();
         jointForceConstraintUpper.addJoint(requestedJoint, forceLimit);
         jointForceConstraintUpper.setConstraintType(ConstraintType.LEQ_INEQUALITY);

         JointTorqueCommand jointForceConstraintLower = new JointTorqueCommand();
         jointForceConstraintLower.addJoint(requestedJoint, -1.0 * forceLimit);
         jointForceConstraintLower.setConstraintType(ConstraintType.GEQ_INEQUALITY);

         // Setup jacobians for validating controller output
         GeometricJacobianCalculator contactingLinkJacobian = new GeometricJacobianCalculator();
         contactingLinkJacobian.setKinematicChain(elevator, contactingLink);
         contactingLinkJacobian.setJacobianFrame(contactingLink.getBodyFixedFrame());

         GeometricJacobianCalculator upperLinkJacobian = new GeometricJacobianCalculator();
         upperLinkJacobian.setKinematicChain(elevator, upperLink);
         upperLinkJacobian.setJacobianFrame(upperLink.getBodyFixedFrame());

         // Run controller core twice - contact state, dynamic matrices and joint torque task aren't processed in order of dependency
         controllerCore.initialize();

         for (int j = 0; j < 2; j++)
         {
            controllerCoreCommand.addInverseDynamicsCommand(jointForceConstraintLower);
            controllerCoreCommand.addInverseDynamicsCommand(jointForceConstraintUpper);
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
            controllerCoreCommand.addInverseDynamicsCommand(contactingLinkAccelerationCommand);
            controllerCoreCommand.addInverseDynamicsCommand(floatingLinkAccelerationCommand);
            controllerCoreCommand.addInverseDynamicsCommand(jointForceCommand);
            controllerCore.compute(controllerCoreCommand);
         }

         JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCore.getOutputForLowLevelController();
         RootJointDesiredConfigurationDataReadOnly rootJointDesiredData = controllerCore.getOutputForRootJoint();
         double desiredForceFromQP = lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(requestedJoint);
         double epsilon = 1e-7;

         // Check that force limits are respected
         Assertions.assertTrue(desiredForceFromQP <= forceLimit + epsilon);
         Assertions.assertTrue(desiredForceFromQP >= -1.0 * forceLimit - epsilon);

         double epsilon2 = 1e-5;
         if (desiredActuatorForce <= forceLimit + epsilon2 && desiredActuatorForce >= -forceLimit - epsilon2)
         {
            DMatrixRMaj contactingLinkAcceleration = computeSpatialAcceleration(contactingLinkJacobian,
                                                                                lowLevelOneDoFJointDesiredDataHolder,
                                                                                rootJointDesiredData);
            DMatrixRMaj upperLinkAcceleration = computeSpatialAcceleration(upperLinkJacobian, lowLevelOneDoFJointDesiredDataHolder, rootJointDesiredData);

            // Check that achieved spatial accelerations are zero if force limit was not hit
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(contactingLinkAcceleration.get(5, 0), 0.0, epsilon2));
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(upperLinkAcceleration.get(5, 0), 0.0, epsilon2));

            // Check that achieved force matches requested if force limit was not hit
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(desiredForceFromQP, desiredActuatorForce, epsilon2));
         }
      }
   }

   private static DMatrixRMaj computeSpatialAcceleration(GeometricJacobianCalculator jacobianCalculator,
                                                         JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder,
                                                         RootJointDesiredConfigurationDataReadOnly rootJointDesiredData)
   {
      List<JointReadOnly> jointChain = jacobianCalculator.getJointsFromBaseToEndEffector();
      DMatrixRMaj jacobianMatrix = jacobianCalculator.getJacobianMatrix();
      DMatrixRMaj convectiveTermMatrix = jacobianCalculator.getConvectiveTermMatrix();
      DMatrixRMaj qdd = new DMatrixRMaj(jacobianCalculator.getNumberOfDegreesOfFreedom(), 1);

      int row = 0;
      for (int i = 0; i < jointChain.size(); i++)
      {
         JointReadOnly joint = jointChain.get(i);
         if (joint instanceof FloatingJointReadOnly)
            MatrixTools.setMatrixBlock(qdd, row, 0, rootJointDesiredData.getDesiredAcceleration(), 0, 0, 6, 1, 1.0);
         else
            qdd.set(row, 0, lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration((OneDoFJointReadOnly) joint));
         row += joint.getDegreesOfFreedom();
      }

      DMatrixRMaj spatialAcceleration = new DMatrixRMaj(6, 1);
      CommonOps_DDRM.mult(jacobianMatrix, qdd, spatialAcceleration);
      CommonOps_DDRM.addEquals(spatialAcceleration, convectiveTermMatrix);
      return spatialAcceleration;
   }

   public class JointTorqueTestOptimizationSettings implements ControllerCoreOptimizationSettings
   {
      private final double jointAccelerationWeight;
      private final int numberOfContactableBodies;

      public JointTorqueTestOptimizationSettings(double jointAccelerationWeight, int numberOfContactableBodies)
      {
         this.jointAccelerationWeight = jointAccelerationWeight;
         this.numberOfContactableBodies = numberOfContactableBodies;
      }

      @Override
      public double getJointAccelerationWeight()
      {
         return jointAccelerationWeight;
      }

      @Override
      public double getJointJerkWeight()
      {
         return 0.0;
      }

      @Override
      public double getRhoWeight()
      {
         return 0;
      }

      @Override
      public double getRhoMin()
      {
         return 0;
      }

      @Override
      public double getRhoRateDefaultWeight()
      {
         return 0;
      }

      @Override
      public Vector2D getCoPWeight()
      {
         return new Vector2D();
      }

      @Override
      public Vector2D getCoPRateDefaultWeight()
      {
         return new Vector2D();
      }

      @Override
      public int getNumberOfBasisVectorsPerContactPoint()
      {
         return 4;
      }

      @Override
      public int getNumberOfContactPointsPerContactableBody()
      {
         return 1;
      }

      @Override
      public int getNumberOfContactableBodies()
      {
         return numberOfContactableBodies;
      }

      @Override
      public boolean updateDynamicMatrixCalculator()
      {
         return true;
      }
   }
}
