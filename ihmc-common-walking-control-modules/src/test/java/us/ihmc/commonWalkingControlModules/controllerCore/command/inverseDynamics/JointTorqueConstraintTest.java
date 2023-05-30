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
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class JointTorqueConstraintTest
{
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
         jointTorqueConstraintLower.addJoint(joint, -1.0*torqueLimit);
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
         
         // Assert joint torque matches requested if it did not exceed the torqueLimit
         double epsilon2 = 1e-5;
         if (desiredTorque <= torqueLimit + epsilon && desiredTorque >= -torqueLimit - epsilon)
            Assertions.assertTrue(EuclidCoreTools.epsilonEquals(desiredTorqueFromQP, desiredTorque, epsilon2));
      }
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
