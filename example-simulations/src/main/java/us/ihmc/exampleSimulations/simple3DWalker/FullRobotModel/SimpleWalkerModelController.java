package us.ihmc.exampleSimulations.simple3DWalker.FullRobotModel;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelController;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SimpleWalkerModelController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("controller");

   private final Map<InverseDynamicsJoint, YoDouble> yoJointTorques = new HashMap<>();

   private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final FullRobotModel robotModel;
   private final OneDoFJoint[] controlledJoints;

   private final VirtualModelController virtualModelController;

   private Wrench desiredWrench = new Wrench();


   private List<YoWrench> yoDesiredWrenches = new ArrayList<>();
   private List<RigidBody> endEffectors = new ArrayList<>();
   private final DenseMatrix64F selectionMatrix;

   SimpleWalkerModelController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel robotModel, OneDoFJoint[] controlledJoints,
                VirtualModelController virtualModelController, List<RigidBody> endEffectors,
                      List<YoWrench> yoDesiredWrenches, DenseMatrix64F selectionMatrix)
   {
      this.scsRobot = scsRobot;
      this.robotModel = robotModel;
      this.controlledJoints = controlledJoints;
      this.virtualModelController = virtualModelController;
      this.endEffectors = endEffectors;
      this.selectionMatrix = selectionMatrix;
      this.yoDesiredWrenches = yoDesiredWrenches;

      for (InverseDynamicsJoint joint : controlledJoints)
         yoJointTorques.put(joint, new YoDouble(joint.getName() + "solutionTorque", registry));

   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      // copy from scs
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();
      scsRobot.update();
      robotModel.updateFrames();

      // compute forces
      VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();

      virtualModelController.clear();
      for (int i = 0; i < endEffectors.size(); i++)
      {
         desiredWrench = yoDesiredWrenches.get(i).getWrench();
         virtualModelController.submitControlledBodyVirtualWrench(endEffectors.get(i), desiredWrench, selectionMatrix);
      }
      virtualModelController.compute(virtualModelControlSolution);

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