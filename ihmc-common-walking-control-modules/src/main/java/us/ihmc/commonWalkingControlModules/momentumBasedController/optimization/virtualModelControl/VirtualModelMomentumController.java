package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualEffortCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInput;
import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.*;

import java.util.List;

public class VirtualModelMomentumController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F tempFullJacobian;
   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(Wrench.SIZE, 12);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(Wrench.SIZE, Wrench.SIZE);

   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tempFullObjective = new DenseMatrix64F(Wrench.SIZE, 1);

   private final DenseMatrix64F fullEffortMatrix;

   private final SelectionMatrix6D defaultSelectionMatrix = new SelectionMatrix6D();

   public VirtualModelMomentumController(JointIndexHandler jointIndexHandler)
   {
      this.jointIndexHandler = jointIndexHandler;

      fullEffortMatrix = new DenseMatrix64F(jointIndexHandler.getNumberOfDoFs(), 1);
      tempFullJacobian = new DenseMatrix64F(Wrench.SIZE, jointIndexHandler.getNumberOfDoFs());
   }

   public void reset()
   {
      fullEffortMatrix.zero();
   }

   /**
    * Adds a {@link VirtualEffortCommand} to the {@link VirtualModelMomentumController}.
    * <p>
    * The idea is to add the data from the virtual effort command so that the virtual model
    * controller exerts the desired forces based on the equation:<br>
    * J<sub>MxN</sub><sup>T</sup> * w<sub>Mx1</sub> = &tau;<sub>Nx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, w is the M-by-1 desired joint effort vector,
    * and &tau; is the N-by-1 torque vector. M is called the task
    * size and N is the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    *
    * @return true if the command was successfully added.
    */
   public boolean addVirtualEffortCommand(VirtualEffortCommand commandToAdd)
   {
      commandToAdd.getControlFrame(controlFrame);
      // Gets the M-by-6 selection matrix S.
      commandToAdd.getSelectionMatrix(controlFrame, tempSelectionMatrix);

      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(commandToAdd.getBase(), commandToAdd.getEndEffector());
      jacobianCalculator.setJacobianFrame(controlFrame);
      jacobianCalculator.computeJacobianMatrix();

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix, j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain, which is way smaller than the number of robot DoFs.
      jacobianCalculator.getJacobianMatrix(tempSelectionMatrix, tempTaskJacobian);

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      List<InverseDynamicsJoint> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, tempFullJacobian);

      /*
       * @formatter:off
       * Compute the N-by-1 task objective vector p as follows:
       * p = S * ( w )
       * where w is the M-by-1 end-effector desired effort vector.
       * @formatter:on
       */
      tempTaskObjective.reshape(taskSize, 1);
      commandToAdd.getDesiredEffort(tempFullObjective);
      CommonOps.mult(tempSelectionMatrix, tempFullObjective, tempTaskObjective);

      // Add these forces to the effort matrix t = J' w
      CommonOps.multAddTransA(tempFullJacobian, tempTaskObjective, fullEffortMatrix);

      return true;
   }

   /**
    * Adds a {@link JointspaceAccelerationCommand} into a {@link MotionQPInput}.
    *
    * @return true if the command was successfully added.
    */
   public boolean addJointTorqueCommand(JointTorqueCommand commandToAdd)
   {
      int taskSize = ScrewTools.computeDegreesOfFreedom(commandToAdd.getJoints());

      if (taskSize == 0)
         return false;

      for (int jointNumber = 0; jointNumber < commandToAdd.getNumberOfJoints(); jointNumber++)
      {
         InverseDynamicsJoint joint = commandToAdd.getJoint(jointNumber);
         int[] jointIndices = jointIndexHandler.getJointIndices(joint);
         if (jointIndices == null)
            return false;

         for (int i = 0; i < jointIndices.length; i++)
         {
            int jointIndex = jointIndices[i];
            fullEffortMatrix.add(jointIndex, 0, commandToAdd.getDesiredTorque(jointNumber).get(i, 0));
         }
      }

      return true;
   }

   /**
    * Adds a {@link Wrench} to the {@link VirtualModelMomentumController}.
    * <p>
    * The idea is to add the data from the contact force optimization module so that
    * the virtual model controller exerts the desired forces based on the equation:<br>
    * J<sub>MxN</sub><sup>T</sup> * w<sub>Mx1</sub> = &tau;<sub>Nx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, w is the M-by-1 desired joint effort vector,
    * and &tau; is the N-by-1 torque vector. M is called the task
    * size and N is the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    *
    * @return true if the wrench was successfully added.
    */
   public boolean addExternalWrench(RigidBody base, RigidBody endEffector, Wrench wrench, SelectionMatrix6D selectionMatrix)
   {
      if (wrench.getLinearPart().length() < 1e-5 && wrench.getAngularPart().length() < 1e-5)
         return false;

      // Gets the M-by-6 selection matrix S.
      selectionMatrix.getCompactSelectionMatrixInFrame(wrench.getExpressedInFrame(), tempSelectionMatrix);

      int taskSize = tempSelectionMatrix.getNumRows();

      if (taskSize == 0)
         return false;

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(base, endEffector);
      jacobianCalculator.setJacobianFrame(wrench.getExpressedInFrame());
      jacobianCalculator.computeJacobianMatrix();

      // Compute the M-by-N task Jacobian: J = S * J
      // Step 1, let's get the 'small' Jacobian matrix, j.
      // It is called small as its number of columns is equal to the number of DoFs to its kinematic chain, which is way smaller than the number of robot DoFs.
      jacobianCalculator.getJacobianMatrix(tempSelectionMatrix, tempTaskJacobian);

      // Step 2: The small Jacobian matrix into the full Jacobian matrix. Proper indexing has to be ensured, so it is handled by the jointIndexHandler.
      List<InverseDynamicsJoint> jointsUsedInTask = jacobianCalculator.getJointsFromBaseToEndEffector();
      jointIndexHandler.compactBlockToFullBlockIgnoreUnindexedJoints(jointsUsedInTask, tempTaskJacobian, tempFullJacobian);

      /*
       * @formatter:off
       * Compute the N-by-1 task objective vector p as follows:
       * p = S * ( w )
       * where w is the M-by-1 end-effector desired effort vector.
       * @formatter:on
       */
      tempTaskObjective.reshape(taskSize, 1);
      wrench.getMatrix(tempFullObjective);
      CommonOps.mult(tempSelectionMatrix, tempFullObjective, tempTaskObjective);

      // Add these forces to the effort matrix t = J' w
      CommonOps.multAddTransA(tempFullJacobian, tempTaskObjective, fullEffortMatrix);

      return true;
   }

   /**
    * Adds a {@link Wrench} to the {@link VirtualModelMomentumController}.
    * <p>
    * The idea is to add the data from the contact force optimization module so that
    * the virtual model controller exerts the desired forces based on the equation:<br>
    * J<sub>MxN</sub><sup>T</sup> * w<sub>Mx1</sub> = &tau;<sub>Nx1</sub> <br>
    * where J is the M-by-N Jacobian matrix, w is the M-by-1 desired joint effort vector,
    * and &tau; is the N-by-1 torque vector. M is called the task
    * size and N is the overall number of degrees of freedom (DoFs) to be controlled.
    * </p>
    *
    * @return true if the wrench was successfully added.
    */
   public boolean addExternalWrench(RigidBody base, RigidBody endEffector, Wrench wrench)
   {
      return addExternalWrench(base, endEffector, wrench, defaultSelectionMatrix);
   }

   public void populateTorqueSolution(VirtualModelControlSolution solutionToPack)
   {
      solutionToPack.setJointTorques(fullEffortMatrix);
   }
}
