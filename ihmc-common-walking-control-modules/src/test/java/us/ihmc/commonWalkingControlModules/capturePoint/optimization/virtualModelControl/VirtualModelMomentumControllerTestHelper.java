package us.ihmc.commonWalkingControlModules.capturePoint.optimization.virtualModelControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelMomentumController;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControllerTestHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VirtualModelMomentumControllerTestHelper
{
   static void createVirtualModelMomentumControlTest(SCSRobotFromInverseDynamicsRobotModel robotModel, FullRobotModel controllerModel, ReferenceFrame centerOfMassFrame,
         List<RigidBody> endEffectors, List<Vector3D> desiredForces, List<Vector3D> desiredTorques, List<ExternalForcePoint> externalForcePoints, SelectionMatrix6D selectionMatrix, SimulationTestingParameters simulationTestingParameters) throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      VirtualModelMomentumController virtualModelController = new VirtualModelMomentumController(new JointIndexHandler(controllerModel.getOneDoFJoints()));

      List<YoWrench> desiredWrenches = new ArrayList<>();
      List<VirtualModelControllerTestHelper.ForcePointController> forcePointControllers = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);
         ReferenceFrame endEffectorFrame = endEffector.getBodyFixedFrame();

         FramePose3D desiredEndEffectorPose = new FramePose3D(endEffectorFrame);
         desiredEndEffectorPose.setToZero();
         desiredEndEffectorPose.changeFrame(ReferenceFrame.getWorldFrame());

         Wrench desiredWrench = new Wrench(endEffectorFrame, endEffectorFrame);
         Vector3D desiredForce = desiredForces.get(i);
         Vector3D desiredTorque = desiredTorques.get(i);
         FrameVector3D forceFrameVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), desiredForce);
         FrameVector3D torqueFrameVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), desiredTorque);
         forceFrameVector.changeFrame(endEffectorFrame);
         torqueFrameVector.changeFrame(endEffectorFrame);
         desiredWrench.set(forceFrameVector, torqueFrameVector);
         desiredWrench.changeFrame(ReferenceFrame.getWorldFrame());

         YoWrench yoDesiredWrench = new YoWrench("desiredWrench" + i, endEffectorFrame, ReferenceFrame.getWorldFrame(), registry);
         yoDesiredWrench.set(desiredWrench);

         desiredWrenches.add(yoDesiredWrench);

         Vector3D contactForce = new Vector3D();
         Vector3D contactTorque = new Vector3D();
         contactForce.set(desiredForce);
         contactTorque.set(desiredTorque);
         contactForce.scale(-1.0);
         contactTorque.scale(-1.0);

         VirtualModelControllerTestHelper.ForcePointController forcePointController = new VirtualModelControllerTestHelper.ForcePointController("" + i, externalForcePoints.get(i), endEffectorFrame, desiredEndEffectorPose);
         forcePointController.setInitialForce(contactForce, contactTorque);
         forcePointController.setLinearGains(250, 10, 0);
         forcePointControllers.add(forcePointController);
      }

      DummyArmMomentumController armController = new DummyArmMomentumController(robotModel, controllerModel, controllerModel.getOneDoFJoints(), forcePointControllers, virtualModelController,
                                                                                endEffectors, desiredWrenches, selectionMatrix);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotModel, simulationTestingParameters);
      robotModel.setController(armController);
      scs.getRootRegistry().addChild(registry);
      for (VirtualModelControllerTestHelper.ForcePointController forcePointController : forcePointControllers)
         yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();

      Vector3D currentPosition = new Vector3D();
      Quaternion currentOrientation = new Quaternion();
      Vector3D currentForce = new Vector3D();
      Vector3D currentTorque = new Vector3D();

      List<Vector3D> desiredPositions = new ArrayList<>();
      List<Quaternion> desiredOrientations = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         desiredPositions.add(armController.getDesiredPosition(i));
         desiredOrientations.add(armController.getDesiredOrientation(i));
      }

      // check that the end effector doesn't move, and that the desired force is very close to what we want
      double timeIncrement = 1.0;
      while (scs.getTime() < simulationDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         for (int i = 0; i < endEffectors.size(); i++)
         {
            currentPosition.set(armController.getCurrentPosition(i));
            currentOrientation.set(armController.getCurrentOrientation(i));
            currentForce.set(armController.getCurrentForce(i));
            currentTorque.set(armController.getCurrentTorque(i));

            EuclidCoreTestTools.assertTuple3DEquals("", currentPosition, desiredPositions.get(i), 0.01);
            EuclidCoreTestTools.assertQuaternionEquals(currentOrientation, desiredOrientations.get(i), 0.01);
            EuclidCoreTestTools.assertTuple3DEquals("", desiredForces.get(i), currentForce, 0.5);
            EuclidCoreTestTools.assertTuple3DEquals("", desiredTorques.get(i), currentTorque, 0.5);
         }
      }

      scs.closeAndDispose();
      blockingSimulationRunner = null;
   }

   static VirtualModelControllerTestHelper.RobotLegs createRobotLeg(double gravity)
   {
      return VirtualModelControllerTestHelper.createRobotLeg(gravity);
   }

   static VirtualModelControllerTestHelper.RobotArm createRobotArm()
   {
      return VirtualModelControllerTestHelper.createRobotArm();
   }

   static VirtualModelControllerTestHelper.PlanarForkedRobotArm createPlanarForkedRobotArm()
   {
      return VirtualModelControllerTestHelper.createPlanarForkedRobotArm();
   }

   static VirtualModelControllerTestHelper.ForkedRobotArm createForkedRobotArm()
   {
      return VirtualModelControllerTestHelper.createForkedRobotArm();
   }

   static VirtualModelControllerTestHelper.PlanarRobotArm createPlanarArm()
   {
      return VirtualModelControllerTestHelper.createPlanarArm();
   }

   static void compareWrenches(Wrench wrench1, Wrench wrench2)
   {
      VirtualModelControllerTestHelper.compareWrenches(wrench1, wrench2);
   }

   static void compareWrenches(Wrench wrench1, Wrench wrench2, SelectionMatrix6D selectionMatrix)
   {
      VirtualModelControllerTestHelper.compareWrenches(wrench1, wrench2, selectionMatrix);
   }


   private static class DummyArmMomentumController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("controller");

      private final Map<InverseDynamicsJoint, YoDouble> yoJointTorques = new HashMap<>();

      private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
      private final FullRobotModel controllerModel;
      private final OneDoFJoint[] controlledJoints;

      private final VirtualModelMomentumController virtualModelController;

      private Wrench desiredWrench = new Wrench();

      private List<VirtualModelControllerTestHelper.ForcePointController> forcePointControllers = new ArrayList<>();
      private List<YoWrench> yoDesiredWrenches = new ArrayList<>();
      private List<RigidBody> endEffectors = new ArrayList<>();
      private final SelectionMatrix6D selectionMatrix;

      DummyArmMomentumController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel controllerModel, OneDoFJoint[] controlledJoints,
                                        List<VirtualModelControllerTestHelper.ForcePointController> forcePointControllers,
                                        VirtualModelMomentumController virtualModelController, List<RigidBody> endEffectors,
                                        List<YoWrench> yoDesiredWrenches, SelectionMatrix6D selectionMatrix)
      {
         this.scsRobot = scsRobot;
         this.controllerModel = controllerModel;
         this.controlledJoints = controlledJoints;
         this.forcePointControllers = forcePointControllers;
         this.virtualModelController = virtualModelController;
         this.endEffectors = endEffectors;
         this.selectionMatrix = selectionMatrix;
         this.yoDesiredWrenches = yoDesiredWrenches;

         for (InverseDynamicsJoint joint : controlledJoints)
            yoJointTorques.put(joint, new YoDouble(joint.getName() + "solutionTorque", registry));

         for (VirtualModelControllerTestHelper.ForcePointController forcePointController : forcePointControllers)
            registry.addChild(forcePointController.getYoVariableRegistry());
      }

      @Override
      public void initialize()
      {
         for (VirtualModelControllerTestHelper.ForcePointController forcePointController : forcePointControllers)
            forcePointController.initialize();
      }

      @Override
      public void doControl()
      {
         // copy from scs
         scsRobot.updateJointPositions_SCS_to_ID();
         scsRobot.updateJointVelocities_SCS_to_ID();
         scsRobot.update();
         controllerModel.updateFrames();

         for (VirtualModelControllerTestHelper.ForcePointController forcePointController : forcePointControllers)
            forcePointController.doControl();

         // compute forces
         VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();

         virtualModelController.reset();
         for (int i = 0; i < endEffectors.size(); i++)
         {
            desiredWrench = yoDesiredWrenches.get(i).getWrench();
            virtualModelController.addExternalWrench(controllerModel.getRootBody(), endEffectors.get(i), desiredWrench, selectionMatrix);
         }
         virtualModelController.populateTorqueSolution(virtualModelControlSolution);

         DenseMatrix64F jointTorques = virtualModelControlSolution.getJointTorques();
         for (int i = 0; i < controlledJoints.length; i++)
         {
            OneDoFJoint joint = controlledJoints[i];
            double tau = jointTorques.get(i, 0);
            yoJointTorques.get(joint).set(tau);
            joint.setTau(tau);
         }

         // write to scs
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
      }

      Vector3D getDesiredPosition(int index)
      {
         return forcePointControllers.get(index).getDesiredPosition();
      }

      Quaternion getDesiredOrientation(int index)
      {
         return forcePointControllers.get(index).getDesiredOrientation();
      }

      Vector3D getCurrentPosition(int index)
      {
         return forcePointControllers.get(index).getCurrentPosition();
      }

      Quaternion getCurrentOrientation(int index)
      {
         return forcePointControllers.get(index).getCurrentOrientation();
      }

      Vector3D getCurrentForce(int index)
      {
         return forcePointControllers.get(index).getCurrentForce();
      }

      Vector3D getCurrentTorque(int index)
      {
         return forcePointControllers.get(index).getCurrentTorque();
      }

      @Override
      public String getName()
      {
         return "robotArmController";
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }
}
