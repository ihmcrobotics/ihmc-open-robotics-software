package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * Equation of dynamics:
 * 
 * <pre>
 * &tau; = H qdd + C + &sum;<sub>i</sub> J<sup>T</sup><sub>i</sub> F<sub>ext,i</sub>
 * </pre>
 * 
 * where:
 * <ul>
 * <li><tt>qdd</tt> and <tt>&tau;</tt> are the vectors of the joint accelerations and torques.
 * <li><tt>H</tt> is the mass-matrix of the multi-body system.
 * <li><tt>C</tt> is the vector of gravity, centrifugal, and Coriolis forces.
 * <li><tt>J<sub>i</sub></tt> is the Jacobian for the end-effector <tt>i</tt> that is subject to an
 * external wrench.
 * <li><tt>F<sup>ext,i</tt> is the external wrench to the end-effector <tt>i</tt>.
 * </ul>
 * In the context of the {@link WholeBodyInverseDynamicsSolver}, some of the external wrenches
 * <tt>F<sup>ext</tt> are part of the optimization, these are formulated as:
 * 
 * <pre>
 * F<sub>ext</sub> = Q &rho;
 * </pre>
 * 
 * This calculator can be used to compute the matrix <tt>J<sup>T</sup> Q</tt> which maps from
 * contact magnitudes <tt>&rho;</tt>. This is necessary for including joint torque as an objective
 * in the optimization.
 */
public class ContactWrenchMatrixCalculator
{
   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final List<RigidBodyBasics> contactableBodies;
   private final JointIndexHandler jointIndexHandler;
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();

   private final RigidBodyBasics rootBody;

   private final int numberOfDoFs;

   private final DMatrixRMaj tmpCompactContactForceJacobianMatrix;
   private final DMatrixRMaj tmpFullContactForceJacobianMatrix;

   public ContactWrenchMatrixCalculator(WholeBodyControlCoreToolbox toolbox)
   {
      this.rootBody = toolbox.getRootBody();
      this.wrenchMatrixCalculator = toolbox.getWrenchMatrixCalculator();
      this.contactableBodies = wrenchMatrixCalculator.getRigidBodies();
      this.jointIndexHandler = toolbox.getJointIndexHandler();

      numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      tmpFullContactForceJacobianMatrix = new DMatrixRMaj(rhoSize, numberOfDoFs);
      tmpCompactContactForceJacobianMatrix = new DMatrixRMaj(rhoSize, numberOfDoFs);
   }

   /**
    * Computes the Jacobian matrix <tt>Q<sup>T</sup>J</tt>.
    * 
    * @param contactForceJacobianToPack
    */
   public void computeContactForceJacobian(DMatrixRMaj contactForceJacobianToPack)
   {
      int contactForceStartIndex = 0;

      for (int bodyIndex = 0; bodyIndex < contactableBodies.size(); bodyIndex++)
      {
         RigidBodyBasics rigidBody = contactableBodies.get(bodyIndex);

         jacobianCalculator.setKinematicChain(rootBody, rigidBody);
         jacobianCalculator.setJacobianFrame(wrenchMatrixCalculator.getJacobianFrame());
         DMatrixRMaj contactableBodyJacobianMatrix = jacobianCalculator.getJacobianMatrix();

         DMatrixRMaj rhoJacobianMatrix = wrenchMatrixCalculator.getRhoJacobianMatrix(rigidBody);

         int rhoSize = rhoJacobianMatrix.getNumCols();

         tmpCompactContactForceJacobianMatrix.reshape(rhoSize, contactableBodyJacobianMatrix.getNumCols());
         CommonOps_DDRM.multTransA(rhoJacobianMatrix, contactableBodyJacobianMatrix, tmpCompactContactForceJacobianMatrix);
         CommonOps_DDRM.changeSign(tmpCompactContactForceJacobianMatrix);

         jointIndexHandler.compactBlockToFullBlock(jacobianCalculator.getJointsFromBaseToEndEffector(),
                                                   tmpCompactContactForceJacobianMatrix,
                                                   tmpFullContactForceJacobianMatrix);
         CommonOps_DDRM.extract(tmpFullContactForceJacobianMatrix, 0, rhoSize, 0, numberOfDoFs, contactForceJacobianToPack, contactForceStartIndex, 0);

         contactForceStartIndex += rhoSize;
      }
   }
}
