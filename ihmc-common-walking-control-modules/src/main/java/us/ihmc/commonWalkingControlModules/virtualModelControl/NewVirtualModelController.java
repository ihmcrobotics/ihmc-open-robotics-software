package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualEffortCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Wrench;

import java.util.List;

public class NewVirtualModelController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F tempFullJacobian = new DenseMatrix64F(Wrench.SIZE, 12);
   private final DenseMatrix64F tempTaskJacobian = new DenseMatrix64F(Wrench.SIZE, 12);
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F(Wrench.SIZE, Wrench.SIZE);

   private final DenseMatrix64F tempTaskObjective = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F tempFullObjective = new DenseMatrix64F(Wrench.SIZE, 1);


   private final DenseMatrix64F fullEffortMatrix;

   public NewVirtualModelController(JointIndexHandler jointIndexHandler)
   {
      this.jointIndexHandler = jointIndexHandler;

      fullEffortMatrix = new DenseMatrix64F(jointIndexHandler.getNumberOfDoFs(), 0);
   }

   public void reset()
   {
      fullEffortMatrix.zero();
   }

   public void addVirtualEffortCommand(VirtualEffortCommand commandToConvert)
   {
      commandToConvert.getControlFrame(controlFrame);
      // Gets the M-by-6 selection matrix S.
      commandToConvert.getSelectionMatrix(controlFrame, tempSelectionMatrix);

      jacobianCalculator.clear();
      jacobianCalculator.setKinematicChain(commandToConvert.getBase(), commandToConvert.getEndEffector());
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
       * Compute the M-by-1 task objective vector p as follows:
       * p = S * ( TDot - JDot * qDot )
       * where TDot is the 6-by-1 end-effector desired acceleration vector and (JDot * qDot) is the 6-by-1
       * convective term vector resulting from the Coriolis and Centrifugal effects.
       * @formatter:on
       */
      commandToConvert.getDesiredEffort(tempTaskObjective);
      CommonOps.mult(tempSelectionMatrix, tempTaskObjective, tempFullObjective);

      // Add these forces to the effort matrix t = J' w
      CommonOps.multAddTransA(tempFullJacobian, tempFullObjective, fullEffortMatrix);
   }
}
