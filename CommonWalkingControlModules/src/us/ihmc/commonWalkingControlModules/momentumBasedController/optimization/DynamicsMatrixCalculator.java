package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.*;

import java.util.ArrayList;

public class DynamicsMatrixCalculator
{
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisMatrixCalculator;

   private final WrenchMatrixCalculator wrenchMatrixCalculator;

   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F coriolisMatrix;
   private final DenseMatrix64F contactForceJacobian;

   private final DenseMatrix64F floatingBaseMassMatrix;
   private final DenseMatrix64F floatingBaseCoriolisMatrix;
   private final DenseMatrix64F floatingBaseContactForceJacobian;

   private final DenseMatrix64F bodyMassMatrix;
   private final DenseMatrix64F bodyCoriolisMatrix;
   private final DenseMatrix64F bodyContactForceJacobian;

   private final int numberOfDoFs;
   private final int floatingBaseDoFs;

   public DynamicsMatrixCalculator(ArrayList<InverseDynamicsJoint> jointsToIgnore, TwistCalculator twistCalculator, WrenchMatrixCalculator wrenchMatrixCalculator)
   {
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;

      RigidBody rootBody = twistCalculator.getRootBody();
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody, jointsToIgnore);
      coriolisMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(jointsToIgnore, twistCalculator, true);

      numberOfDoFs = coriolisMatrixCalculator.getNumberOfDegreesOfFreedom();
      floatingBaseDoFs = rootBody.getParentJoint().getDegreesOfFreedom();
      int bodyDoFs = numberOfDoFs - floatingBaseDoFs;

      massMatrix = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      coriolisMatrix = new DenseMatrix64F(numberOfDoFs, 1);
      contactForceJacobian = new DenseMatrix64F(numberOfDoFs, rhoSize);

      floatingBaseMassMatrix = new DenseMatrix64F(floatingBaseDoFs, numberOfDoFs);
      floatingBaseCoriolisMatrix = new DenseMatrix64F(floatingBaseDoFs, 1);
      floatingBaseContactForceJacobian = new DenseMatrix64F(floatingBaseDoFs, rhoSize);

      bodyMassMatrix = new DenseMatrix64F(bodyDoFs, numberOfDoFs);
      bodyCoriolisMatrix = new DenseMatrix64F(bodyDoFs, 1);
      bodyContactForceJacobian = new DenseMatrix64F(bodyDoFs, rhoSize);
   }

   public void reset()
   {
      coriolisMatrixCalculator.reset();
   }

   public void compute()
   {
      massMatrixCalculator.compute();
      coriolisMatrixCalculator.compute();

      updateMatrices();
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

   private void updateMatrices()
   {
      massMatrixCalculator.getMassMatrix(massMatrix);
      CommonOps.extract(massMatrix, 0, floatingBaseDoFs, 0, numberOfDoFs, floatingBaseMassMatrix, 0, 0);
      CommonOps.extract(massMatrix, floatingBaseDoFs, numberOfDoFs, 0, numberOfDoFs, bodyMassMatrix, 0, 0);

      coriolisMatrixCalculator.getCoriolisMatrix(coriolisMatrix);
      coriolisMatrixCalculator.getFloatingBaseCoriolisMatrix(floatingBaseCoriolisMatrix);
      coriolisMatrixCalculator.getBodyCoriolisMatrix(bodyCoriolisMatrix);

      wrenchMatrixCalculator.getRhoJacobianMatrix(contactForceJacobian);
      CommonOps.extract(contactForceJacobian, 0, floatingBaseDoFs, 0, numberOfDoFs, floatingBaseContactForceJacobian, 0, 0);
      CommonOps.extract(contactForceJacobian, floatingBaseDoFs, numberOfDoFs, 0, numberOfDoFs, bodyContactForceJacobian, 0, 0);
   }

   public void getFloatingBaseMassMatrix(DenseMatrix64F floatingBaseMassMatrixToPack)
   {
      floatingBaseMassMatrixToPack.set(floatingBaseMassMatrix);
   }

   public void getFloatingCoriolisMatrix(DenseMatrix64F floatingBaseCoriolisMatrixToPack)
   {
      floatingBaseCoriolisMatrixToPack.set(floatingBaseCoriolisMatrix);
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
      jointTorquesToPack.zero();

      CommonOps.mult(-1.0, bodyContactForceJacobian, contactForceSolution, jointTorquesToPack);
      CommonOps.addEquals(jointTorquesToPack, bodyCoriolisMatrix);
      CommonOps.multAdd(bodyMassMatrix, jointAccelerationSolution, jointTorquesToPack);
   }
}
