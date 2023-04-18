package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

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
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

public class JointTorqueCommandTest
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final RigidBodyBasics elevator;
   private final OneDoFJointBasics joint1;
   private final RigidBodyBasics link1;
   private final OneDoFJointBasics joint2;
   private final RigidBodyBasics link2;
   private final OneDoFJointBasics[] controlledJoints;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
   private final WholeBodyControllerCore controllerCore;

   public JointTorqueCommandTest()
   {
      double linkLength = 1.0;

      elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      joint1 = new RevoluteJoint("joint1", elevator, Axis3D.X);
      link1 = new RigidBody("link1", joint1, 1.0, 0.2, 0.2, 1.0, new Point3D(0.0, 0.0, -0.5 * linkLength));
      joint2 = new RevoluteJoint("joint2", link1, new RigidBodyTransform(new Quaternion(), new Point3D(0.0, 0.0, -linkLength)), Axis3D.X);
      link2 = new RigidBody("link2", joint2, 1.0, 0.2, 0.2, 1.0, new Point3D(0.0, 0.0, -0.5 * linkLength));
      controlledJoints = new OneDoFJointBasics[]{joint1, joint2};

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(0.01,
                                                                                       9.81,
                                                                                       null,
                                                                                       controlledJoints,
                                                                                       centerOfMassFrame,
                                                                                       new JointTorqueTestOptimizationSettings(),
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);
      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(controlledJoints);

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      spatialAccelerationCommand.set(elevator, link2);

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, new FeedbackControllerTemplate(allPossibleCommands), lowLevelControllerCoreOutput, registry);

      elevator.updateFramesRecursively();
      centerOfMassFrame.update();

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.setLinearAxisSelection(false, true, false);
      selectionMatrix.setAngularAxisSelection(false, false, false);
      DefaultPID3DGains positionGains = new DefaultPID3DGains();
      positionGains.setProportionalGains(0.0, 1.0, 0.0);
      positionGains.setDerivativeGains(0.0, 1.0, 0.0);

      FramePose3D controlFramePose = new FramePose3D(link2.getBodyFixedFrame());
      controlFramePose.setZ(- linkLength / 2.0);

      spatialAccelerationCommand.getControlFramePose().setIncludingFrame(controlFramePose);
      spatialAccelerationCommand.getSelectionMatrix().setLinearAxisSelection(false, true, false);
      spatialAccelerationCommand.getSelectionMatrix().setAngularAxisSelection(false, false, false);
      spatialAccelerationCommand.setAsHardConstraint();
      spatialAccelerationCommand.getDesiredLinearAcceleration().setY(1.0);

      controllerCoreCommand.addInverseDynamicsCommand(spatialAccelerationCommand);

      JointTorqueCommand jointTorqueCommand = new JointTorqueCommand();
      double jointTorqueWeight = 1000.0;
      jointTorqueCommand.addJoint(joint2, 0.0, jointTorqueWeight);
      controllerCoreCommand.addInverseDynamicsCommand(jointTorqueCommand);

      controllerCore.initialize();
      controllerCore.compute(controllerCoreCommand);

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

      LogTools.info("Computed accelerations:");
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(0));
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointAcceleration(1));

      LogTools.info("Computed torques:");
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(0));
      System.out.println(lowLevelOneDoFJointDesiredDataHolder.getDesiredJointTorque(1));
   }

   public class JointTorqueTestOptimizationSettings implements ControllerCoreOptimizationSettings
   {
      @Override
      public double getJointAccelerationWeight()
      { // only minimize joint acceleration at unit cost
         return 1.0;
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
         return 0;
      }

      @Override
      public int getNumberOfContactPointsPerContactableBody()
      {
         return 0;
      }

      @Override
      public int getNumberOfContactableBodies()
      {
         return 0;
      }
   }


   public static void main(String[] args)
   {
      new JointTorqueCommandTest();
   }
}
