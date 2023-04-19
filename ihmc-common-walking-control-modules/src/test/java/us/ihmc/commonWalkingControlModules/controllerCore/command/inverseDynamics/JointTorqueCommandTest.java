package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
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
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class JointTorqueCommandTest
{
   @Test
   public void testJointTorqueCommandOnDoublePendulum()
   {
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(name);

      double linkLength = 1.0;

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
                                                                                       new JointTorqueTestOptimizationSettings(0),
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
         DMatrixRMaj jacobianMatrix = jacobianCalculator.getJacobianMatrix();
         DMatrixRMaj convectiveTermMatrix = jacobianCalculator.getConvectiveTermMatrix();
         DMatrixRMaj qdd = new DMatrixRMaj(2, 1);
         qdd.set(0, 0, lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(0));
         qdd.set(1, 0, lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(1));
         DMatrixRMaj spatialAcceleration = new DMatrixRMaj(6, 1);
         CommonOps_DDRM.mult(jacobianMatrix, qdd, spatialAcceleration);
         CommonOps_DDRM.addEquals(spatialAcceleration, convectiveTermMatrix);
         Assertions.assertTrue(EuclidCoreTools.epsilonEquals(spatialAcceleration.get(4, 0), 1.0, 1e-12));
      }
   }

   @Test
   public void testJointTorqueCommandOnPrismaticMechanism()
   {
      String name = getClass().getSimpleName();
      YoRegistry registry = new YoRegistry(name);

      double jointOffset = 1.0;

      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint floatingJoint = new SixDoFJoint("floatingJoint", elevator);
      RigidBody link1 = new RigidBody("link1", floatingJoint, 0.1, 0.1, 0.1, 1.0, new Point3D());
      PrismaticJoint joint1 = new PrismaticJoint("joint1", link1, new Point3D(0.0, 0.0, -0.5 * jointOffset), Axis3D.Z);
      RigidBody link2 = new RigidBody("link2", joint1, 0.1, 0.1, 0.1, 1.0, new Point3D(0.0, 0.0, -0.5 * jointOffset));
      PrismaticJoint joint2 = new PrismaticJoint("joint2", link2, new Point3D(0.0, 0.0, -jointOffset), Axis3D.Z);
      RigidBody link3 = new RigidBody("link3", joint2, 0.1, 0.1, 0.1, 1.0, new Point3D(0.0, 0.0, -0.5 * jointOffset));
      floatingJoint.getJointPose().getPosition().setZ(2.0);
      elevator.updateFramesRecursively();

      OneDoFJointBasics[] controlledJoints = new OneDoFJointBasics[] {joint1, joint2};

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", link1.getBodyFixedFrame());

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(0.01, 9.81, floatingJoint, controlledJoints, centerOfMassFrame, new JointTorqueTestOptimizationSettings(1), yoGraphicsListRegistry, registry);

      List<ContactablePlaneBody> contactableBodies = new ArrayList<>();
      contactableBodies.add(new ListOfPointsContactablePlaneBody(link3, link3.getBodyFixedFrame(), Arrays.asList(new Point2D())));

      controlCoreToolbox.setupForInverseDynamicsSolver(contactableBodies);
      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(controlledJoints);

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(controlCoreToolbox, new FeedbackControllerTemplate(allPossibleCommands), lowLevelControllerCoreOutput, registry);

      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
      planeContactStateCommand.setContactingRigidBody(link3);
      planeContactStateCommand.setCoefficientOfFriction(1.0);
      planeContactStateCommand.setContactNormal(new FrameVector3D(link3.getBodyFixedFrame(), 0.0, 0.0, 1.0));
      planeContactStateCommand.setHasContactStateChanged(true);
      planeContactStateCommand.addPointInContact(new FramePoint3D(link3.getBodyFixedFrame()));
      planeContactStateCommand.setRhoWeight(0, 0.0);

      ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
      spatialAccelerationCommand.getSelectionMatrix().setLinearAxisSelection(false, false, true);
      spatialAccelerationCommand.getSelectionMatrix().setAngularAxisSelection(false, false, false);
      spatialAccelerationCommand.setAsHardConstraint();
      spatialAccelerationCommand.getDesiredLinearAcceleration().setZ(1.0);
      spatialAccelerationCommand.set(elevator, link1);

      elevator.updateFramesRecursively();
      controllerCoreCommand.addInverseDynamicsCommand(spatialAccelerationCommand);
      controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);

      controllerCore.initialize();
      controllerCore.compute(controllerCoreCommand);
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCore.getOutputForLowLevelController();

      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(0));
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(1));
      System.out.println();
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(0));
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(1));
   }

   public class JointTorqueTestOptimizationSettings implements ControllerCoreOptimizationSettings
   {
      private final int numberOfContactableBodies;

      public JointTorqueTestOptimizationSettings(int numberOfContactableBodies)
      {
         this.numberOfContactableBodies = numberOfContactableBodies;
      }

      @Override
      public double getJointAccelerationWeight()
      { // only minimize joint acceleration at unit cost
         return 0.1;
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
   }
}
