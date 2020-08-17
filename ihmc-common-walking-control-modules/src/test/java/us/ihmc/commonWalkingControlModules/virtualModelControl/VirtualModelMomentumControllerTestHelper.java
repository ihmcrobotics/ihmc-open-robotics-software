package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelMomentumController;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VirtualModelMomentumControllerTestHelper
{
   static void createVirtualModelMomentumControlTest(SCSRobotFromInverseDynamicsRobotModel robotModel, FullRobotModel controllerModel, ReferenceFrame centerOfMassFrame,
         List<RigidBodyBasics> endEffectors, List<Vector3D> desiredForces, List<Vector3D> desiredTorques, List<ExternalForcePoint> externalForcePoints, SelectionMatrix6D selectionMatrix, SimulationTestingParameters simulationTestingParameters) throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoRegistry registry = new YoRegistry("robert");

      VirtualModelMomentumController virtualModelController = new VirtualModelMomentumController(new JointIndexHandler(controllerModel.getOneDoFJoints()));

      List<YoFixedFrameWrench> desiredWrenches = new ArrayList<>();
      List<VirtualModelControllerTestHelper.ForcePointController> forcePointControllers = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBodyBasics endEffector = endEffectors.get(i);
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
         desiredWrench.set(torqueFrameVector, forceFrameVector);
         desiredWrench.changeFrame(ReferenceFrame.getWorldFrame());

         YoFixedFrameWrench yoDesiredWrench = new YoFixedFrameWrench("desiredWrench" + i, endEffectorFrame, ReferenceFrame.getWorldFrame(), registry);
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

   static void compareWrenches(WrenchReadOnly wrench1, Wrench wrench2)
   {
      VirtualModelControllerTestHelper.compareWrenches(wrench1, wrench2);
   }

   static void compareWrenches(WrenchReadOnly wrench1, Wrench wrench2, SelectionMatrix6D selectionMatrix)
   {
      VirtualModelControllerTestHelper.compareWrenches(wrench1, wrench2, selectionMatrix);
   }


   private static class DummyArmMomentumController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry("controller");

      private final Map<JointBasics, YoDouble> yoJointTorques = new HashMap<>();

      private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
      private final FullRobotModel controllerModel;
      private final OneDoFJointBasics[] controlledJoints;

      private final VirtualModelMomentumController virtualModelController;

      private Wrench desiredWrench = new Wrench();

      private List<VirtualModelControllerTestHelper.ForcePointController> forcePointControllers = new ArrayList<>();
      private List<YoFixedFrameWrench> yoDesiredWrenches = new ArrayList<>();
      private List<RigidBodyBasics> endEffectors = new ArrayList<>();
      private final SelectionMatrix6D selectionMatrix;

      DummyArmMomentumController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel controllerModel, OneDoFJointBasics[] controlledJoints,
                                        List<VirtualModelControllerTestHelper.ForcePointController> forcePointControllers,
                                        VirtualModelMomentumController virtualModelController, List<RigidBodyBasics> endEffectors,
                                        List<YoFixedFrameWrench> yoDesiredWrenches, SelectionMatrix6D selectionMatrix)
      {
         this.scsRobot = scsRobot;
         this.controllerModel = controllerModel;
         this.controlledJoints = controlledJoints;
         this.forcePointControllers = forcePointControllers;
         this.virtualModelController = virtualModelController;
         this.endEffectors = endEffectors;
         this.selectionMatrix = selectionMatrix;
         this.yoDesiredWrenches = yoDesiredWrenches;

         for (JointBasics joint : controlledJoints)
            yoJointTorques.put(joint, new YoDouble(joint.getName() + "solutionTorque", registry));

         for (VirtualModelControllerTestHelper.ForcePointController forcePointController : forcePointControllers)
            registry.addChild(forcePointController.getYoRegistry());
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
            desiredWrench.setIncludingFrame(yoDesiredWrenches.get(i));
            virtualModelController.addExternalWrench(controllerModel.getRootBody(), endEffectors.get(i), desiredWrench, selectionMatrix);
         }
         virtualModelController.populateTorqueSolution(virtualModelControlSolution);

         DMatrixRMaj jointTorques = virtualModelControlSolution.getJointTorques();
         for (int i = 0; i < controlledJoints.length; i++)
         {
            OneDoFJointBasics joint = controlledJoints[i];
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
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }
}
