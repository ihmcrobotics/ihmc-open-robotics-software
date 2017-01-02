package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.*;

import java.util.ArrayList;
import java.util.List;

public class DynamicsMatrixCalculator
{
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;

   private final DynamicsMatrixCalculatorHelper helper;

   private final WholeBodyControlCoreToolbox toolbox;
   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final GeometricJacobianHolder geometricJacobianHolder;
   private final JointIndexHandler jointIndexHandler;

   private final DenseMatrix64F coriolisMatrix;
   private final DenseMatrix64F contactForceJacobian;

   private final DenseMatrix64F floatingBaseMassMatrix;
   private final DenseMatrix64F floatingBaseCoriolisMatrix;
   private final DenseMatrix64F floatingBaseContactForceJacobian;

   private final DenseMatrix64F bodyMassMatrix;
   private final DenseMatrix64F bodyCoriolisMatrix;
   private final DenseMatrix64F bodyContactForceJacobian;

   private final DenseMatrix64F jointTorques;

   private final FloatingInverseDynamicsJoint floatingJoint;
   private final RigidBody elevator;
   private final int numberOfDoFs;
   private final int floatingBaseDoFs;
   private final int rhoSize;

   private final DenseMatrix64F tmpFullContactJacobianMatrix;
   private final DenseMatrix64F tmpContactJacobianMatrixTranspose;
   private final DenseMatrix64F tmpContactJacobianMatrix;

   public DynamicsMatrixCalculator(WholeBodyControlCoreToolbox toolbox, WrenchMatrixCalculator wrenchMatrixCalculator)
   {
      this(new ArrayList<>(), toolbox, wrenchMatrixCalculator);
   }

   public DynamicsMatrixCalculator(ArrayList<InverseDynamicsJoint> jointsToIgnore, WholeBodyControlCoreToolbox toolbox, WrenchMatrixCalculator wrenchMatrixCalculator)
   {
      this.toolbox = toolbox;
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;
      this.geometricJacobianHolder = toolbox.getGeometricJacobianHolder();
      this.jointIndexHandler = toolbox.getJointIndexHandler();

      FullRobotModel fullRobotModel = toolbox.getFullRobotModel();
      elevator = fullRobotModel.getElevator();
      rhoSize = wrenchMatrixCalculator.getRhoSize();

      numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      floatingJoint = toolbox.getRobotRootJoint();
      floatingBaseDoFs = floatingJoint.getDegreesOfFreedom();

      massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(toolbox.getCenterOfMassFrame(), elevator, jointsToIgnore);
      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(toolbox.getTwistCalculator(), toolbox.getGravityZ(), jointsToIgnore);

      helper = new DynamicsMatrixCalculatorHelper(coriolisMatrixCalculator, jointIndexHandler);

      int bodyDoFs = numberOfDoFs - floatingBaseDoFs;

      jointTorques = new DenseMatrix64F(bodyDoFs, 1);

      coriolisMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      contactForceJacobian = new DenseMatrix64F(rhoSize, numberOfDoFs);

      floatingBaseMassMatrix = new DenseMatrix64F(floatingBaseDoFs, numberOfDoFs);
      floatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseDoFs, 1);
      floatingBaseContactForceJacobian = new DenseMatrix64F(rhoSize, floatingBaseDoFs);

      bodyMassMatrix = new DenseMatrix64F(bodyDoFs, numberOfDoFs);
      bodyCoriolisMatrix = new DenseMatrix64F(bodyDoFs, 1);
      bodyContactForceJacobian = new DenseMatrix64F(rhoSize, bodyDoFs);

      tmpFullContactJacobianMatrix = new DenseMatrix64F(rhoSize, numberOfDoFs);
      tmpContactJacobianMatrixTranspose = new DenseMatrix64F(numberOfDoFs, rhoSize);
      tmpContactJacobianMatrix = new DenseMatrix64F(rhoSize, numberOfDoFs);
   }

   public void reset()
   {
      coriolisMatrixCalculator.reset();
   }

   public void compute()
   {
      massMatrixCalculator.compute();
      coriolisMatrixCalculator.compute();

      computeMatrices();
   }

   /**
    * <p>
    * Sets an external force to be achieved. This is not a contactable body to use for balancing.
    * </p>
    *
    * @param rigidBody body to which the wrench is applied.
    * @param externalWrench external wrench acting on body.
    */
   public void setExternalWrench(RigidBody rigidBody, Wrench externalWrench)
   {
      coriolisMatrixCalculator.setExternalWrench(rigidBody, externalWrench);
   }

   private void computeMatrices()
   {
      int[] floatingBaseIndices = jointIndexHandler.getJointIndices(floatingJoint);
      int floatingBaseStartIndex = floatingBaseIndices[0];
      int floatingBaseEndIndex = floatingBaseIndices[floatingBaseIndices.length - 1];

      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      CommonOps.extract(massMatrix, floatingBaseStartIndex, floatingBaseEndIndex + 1, 0, numberOfDoFs, floatingBaseMassMatrix, 0, 0); // // TODO: 1/2/17 push into helper
      CommonOps.extract(massMatrix, floatingBaseEndIndex + 1, numberOfDoFs, 0, numberOfDoFs, bodyMassMatrix, 0, 0); // // TODO: 1/2/17 push into helper

      helper.computeCoriolisMatrix(coriolisMatrix);
      helper.extractFloatingBaseCoriolisMatrix(floatingJoint, coriolisMatrix, floatingBaseCoriolisMatrix);
      helper.extractBodyCoriolisMatrix(jointIndexHandler.getIndexedOneDoFJoints(), coriolisMatrix, bodyCoriolisMatrix);

      computeContactForceJacobians(); // // TODO: 1/2/17 push into helper
      CommonOps.extract(contactForceJacobian, 0, rhoSize, floatingBaseStartIndex, floatingBaseEndIndex + 1, floatingBaseContactForceJacobian, 0, 0); // // TODO: 1/2/17  push into helper
      CommonOps.extract(contactForceJacobian, 0, rhoSize, floatingBaseEndIndex + 1, numberOfDoFs, bodyContactForceJacobian, 0, 0); // // TODO: 1/2/17  push into helper
   }

   private void computeContactForceJacobians()
   {
      int contactForceStartIndex = 0;
      List<? extends ContactablePlaneBody> contactablePlaneBodies = toolbox.getContactablePlaneBodies();
      for (int bodyIndex = 0; bodyIndex < contactablePlaneBodies.size(); bodyIndex++)
      {
         RigidBody rigidBody = contactablePlaneBodies.get(bodyIndex).getRigidBody();
         long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(elevator, rigidBody, wrenchMatrixCalculator.getJacobianFrame());
         GeometricJacobian geometricJacobian = geometricJacobianHolder.getJacobian(jacobianID);

         DenseMatrix64F contactableBodyJacobianMatrix = geometricJacobian.getJacobianMatrix();
         DenseMatrix64F rhoJacobianMatrix = wrenchMatrixCalculator.getRhoJacobianMatrix(rigidBody);

         int rhoSize = rhoJacobianMatrix.getNumCols();

         tmpContactJacobianMatrixTranspose.reshape(contactableBodyJacobianMatrix.getNumCols(), rhoSize);
         tmpContactJacobianMatrix.reshape(rhoSize, contactableBodyJacobianMatrix.getNumCols());
         CommonOps.multTransA(contactableBodyJacobianMatrix, rhoJacobianMatrix, tmpContactJacobianMatrixTranspose);
         CommonOps.transpose(tmpContactJacobianMatrixTranspose, tmpContactJacobianMatrix);

         jointIndexHandler.compactBlockToFullBlock(geometricJacobian.getJointsInOrder(), tmpContactJacobianMatrix, tmpFullContactJacobianMatrix);
         CommonOps.extract(tmpFullContactJacobianMatrix, 0, rhoSize, 0, numberOfDoFs, contactForceJacobian, contactForceStartIndex, 0);

         contactForceStartIndex += rhoSize;
      }
   }

   public void getFloatingBaseMassMatrix(DenseMatrix64F floatingBaseMassMatrixToPack)
   {
      floatingBaseMassMatrixToPack.set(floatingBaseMassMatrix);
   }

   public DenseMatrix64F getFloatingBaseMassMatrix()
   {
      return floatingBaseMassMatrix;
   }

   public void getFloatingBaseCoriolisMatrix(DenseMatrix64F floatingBaseCoriolisMatrixToPack)
   {
      floatingBaseCoriolisMatrixToPack.set(floatingBaseCoriolisMatrix);
   }

   public DenseMatrix64F getFloatingBaseCoriolisMatrix()
   {
      return floatingBaseCoriolisMatrix;
   }

   public void getFloatingBaseContactForceJacobian(DenseMatrix64F floatingBaseContactForceJacobianToPack)
   {
      floatingBaseContactForceJacobianToPack.set(floatingBaseContactForceJacobian);
   }

   public void getBodyMassMatrix(DenseMatrix64F bodyMassMatrixToPack)
   {
      bodyMassMatrixToPack.set(bodyMassMatrix);
   }

   public void getBodyCoriolisMatrix(DenseMatrix64F bodyCoriolisMatrixToPack)
   {
      bodyCoriolisMatrixToPack.set(bodyCoriolisMatrix);
   }

   public void getBodyContactForceJacobian(DenseMatrix64F bodyContactForceJacobianToPack)
   {
      bodyContactForceJacobianToPack.set(bodyContactForceJacobian);
   }

   public DenseMatrix64F computeJointTorques(DenseMatrix64F jointAccelerationSolution, DenseMatrix64F contactForceSolution)
   {
      computeJointTorques(jointTorques, jointAccelerationSolution, contactForceSolution);

      return jointTorques;
   }

   /**
    * <p>
    * Computes the joint torques that satisfy the rigid body dynamics for the desired joint accelerations and contact forces
    * </p>
    *
    * @param jointTorquesToPack
    * @param jointAccelerationSolution
    * @param contactForceSolution
    */
   public void computeJointTorques(DenseMatrix64F jointTorquesToPack, DenseMatrix64F jointAccelerationSolution, DenseMatrix64F contactForceSolution)
   {
      FloatingBaseRigidBodyDynamicsTools.computeTauGivenRhoAndQddot(bodyMassMatrix, bodyCoriolisMatrix, bodyContactForceJacobian, jointAccelerationSolution,
            contactForceSolution, jointTorquesToPack);
   }
}

