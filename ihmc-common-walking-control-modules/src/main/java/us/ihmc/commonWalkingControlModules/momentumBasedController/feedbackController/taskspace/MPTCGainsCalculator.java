package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;

public class MPTCGainsCalculator
{
   private final RigidBodyBasics rootBody;
   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final CompositeRigidBodyMassMatrixCalculator inertiaMatrixCalculator;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisCalculator;
   private final JointIndexHandler jointIndexHandler;
   private final int numberOfDoFs;

   public MPTCGainsCalculator(RigidBodyBasics rootBody)
   {
      this.rootBody = rootBody;
      JointBasics[] joints = MultiBodySystemTools.collectSubtreeJoints(rootBody);
      jointIndexHandler = new JointIndexHandler(joints);
      numberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(joints);
      inertiaMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody);
      coriolisCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(rootBody);
   }

   private final DMatrixRMaj proportionalGains = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj derivativeGains = new DMatrixRMaj(6, 6);

   public void computeMPTCGains(PIDSE3Gains gains, RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      computeMPTCGains(gains.getOrientationGains(), gains.getPositionGains(), endEffector, controlFrame);
   }

   public void computeMPTCGains(PID3DGains angularGains, PID3DGains linearGains, RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      inertiaMatrixCalculator.reset();
      coriolisCalculator.compute();

      proportionalGains.zero();
      derivativeGains.zero();

      for (int i = 0; i < 3; i++)
      {
         if (angularGains != null)
         {
            proportionalGains.set(i, i, angularGains.getProportionalGains()[i]);
            derivativeGains.set(i, i, angularGains.getDerivativeGains()[i]);
         }
         else
         {
            proportionalGains.set(i, i, 0);
            derivativeGains.set(i, i, 0);
         }

         if (linearGains != null)
         {
            proportionalGains.set(i + 3, i + 3, linearGains.getProportionalGains()[i]);
            derivativeGains.set(i + 3, i + 3, linearGains.getDerivativeGains()[i]);
         }
         else
         {
            proportionalGains.set(i + 3, i + 3, 0);
            derivativeGains.set(i + 3, i + 3, 0);
         }
      }

      DMatrixRMaj Ck = computeTaskSpaceCoriolisAndCentrifugalMatrix(endEffector, controlFrame);
      CommonOps_DDRM.addEquals(derivativeGains, Ck);
      DMatrixRMaj Mk_inv = computeEndEffectorInertiaMatrixInverse(endEffector, controlFrame);

      DMatrixRMaj temp = new DMatrixRMaj(6, 6);
      CommonOps_DDRM.mult(Mk_inv, proportionalGains, temp);
      proportionalGains.set(temp);
      CommonOps_DDRM.mult(Mk_inv, derivativeGains, temp);
      derivativeGains.set(temp);
   }

   public DMatrixRMaj getProportionalGains()
   {
      return proportionalGains;
   }

   public DMatrixRMaj getDerivativeGains()
   {
      return derivativeGains;
   }

   /**
    * <pre>
    * M<sub>k</sub> = (J<sub>k</sub>M<sup>-1</sup>J<sub>k</sub><sup>T</sup>)<sup>-1</sup>
    * </pre>
    * 
    * Equation (9)
    */
   private DMatrixRMaj computeEndEffectorInertiaMatrix(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      DMatrixRMaj M = computeEndEffectorInertiaMatrixInverse(endEffector, controlFrame);
      DMatrixRMaj M_inv = new DMatrixRMaj(numberOfDoFs, numberOfDoFs);
      CommonOps_DDRM.pinv(M, M_inv);
      return M_inv;
   }

   /**
    * <pre>
    * M<sub>k</sub><sup>-1</sup> = J<sub>k</sub>M<sup>-1</sup>J<sub>k</sub><sup>T</sup>
    * </pre>
    */
   private DMatrixRMaj computeEndEffectorInertiaMatrixInverse(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      DMatrixRMaj J = computeEndEffectorJacobian(endEffector, controlFrame);
      DMatrixRMaj M_inv = computeSystemInertiaMatrixInverse();
      return CommonOps_DDRM.transpose(MatrixTools.multQuad(CommonOps_DDRM.transpose(J, null), M_inv, null), null);
   }

   /**
    * <tt>M</tt> as defined in Equation (1)
    */
   private DMatrixRMaj computeSystemInertiaMatrixInverse()
   {
      DMatrixRMaj inertiaMatrixInverse = new DMatrixRMaj(numberOfDoFs, numberOfDoFs);
      CommonOps_DDRM.invert(inertiaMatrixCalculator.getMassMatrix(), inertiaMatrixInverse);
      return inertiaMatrixInverse;
   }

   /**
    * <tt>J<sub>k</sub></tt> as defined in Equation (4)
    */
   private DMatrixRMaj computeEndEffectorJacobian(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      geometricJacobianCalculator.setKinematicChain(rootBody, endEffector);
      geometricJacobianCalculator.setJacobianFrame(controlFrame);
      DMatrixRMaj jacobianMatrix = new DMatrixRMaj(geometricJacobianCalculator.getJacobianMatrix().getNumRows(), numberOfDoFs);
      jointIndexHandler.compactBlockToFullBlock(geometricJacobianCalculator.getJointsFromBaseToEndEffector(),
                                                geometricJacobianCalculator.getJacobianMatrix(),
                                                jacobianMatrix);
      return jacobianMatrix;
   }

   /**
    * <tt>dJ<sub>k</sub>/dt</tt> as first introduced in Equation (5)
    */
   private DMatrixRMaj computeEndEffectorJacobianRate(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      geometricJacobianCalculator.setKinematicChain(rootBody, endEffector);
      geometricJacobianCalculator.setJacobianFrame(controlFrame);
      DMatrixRMaj initialJointVelocities = new DMatrixRMaj(geometricJacobianCalculator.getNumberOfDegreesOfFreedom(), 1);
      JointBasics[] jacobianJoints = MultiBodySystemTools.createJointPath(rootBody, endEffector);
      MultiBodySystemTools.extractJointsState(jacobianJoints, JointStateType.VELOCITY, initialJointVelocities);

      DMatrixRMaj compactJacobianRateMatrix = new DMatrixRMaj(6, geometricJacobianCalculator.getNumberOfDegreesOfFreedom());
      DMatrixRMaj jointVelocitiesUnitary = new DMatrixRMaj(geometricJacobianCalculator.getNumberOfDegreesOfFreedom(), 1);

      for (int i = 0; i < geometricJacobianCalculator.getNumberOfDegreesOfFreedom(); i++)
      {
         jointVelocitiesUnitary.zero();
         jointVelocitiesUnitary.set(i, 0, 1.0);
         MultiBodySystemTools.insertJointsState(jacobianJoints, JointStateType.VELOCITY, jointVelocitiesUnitary);
         rootBody.updateFramesRecursively();
         geometricJacobianCalculator.reset();
         CommonOps_DDRM.insert(geometricJacobianCalculator.getConvectiveTermMatrix(), compactJacobianRateMatrix, 0, i);
      }
      MultiBodySystemTools.insertJointsState(jacobianJoints, JointStateType.VELOCITY, initialJointVelocities);
      rootBody.updateFramesRecursively();
      DMatrixRMaj fullJacobianRateMatrix = new DMatrixRMaj(6, numberOfDoFs);
      jointIndexHandler.compactBlockToFullBlock(geometricJacobianCalculator.getJointsFromBaseToEndEffector(),
                                                compactJacobianRateMatrix,
                                                fullJacobianRateMatrix);
      return fullJacobianRateMatrix;
   }

   /**
    * <pre>
    * Q<sub>k</sub> = J<sub>k</sub> M<sup>-1
    * </pre>
    * 
    * Equation (6)
    */
   private DMatrixRMaj computeQk(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      DMatrixRMaj Jk = computeEndEffectorJacobian(endEffector, controlFrame);
      DMatrixRMaj Minv = computeSystemInertiaMatrixInverse();
      DMatrixRMaj C = computeSystemCoriolisAndCentrifugalMatrix();
      DMatrixRMaj JkDot = computeEndEffectorJacobianRate(endEffector, controlFrame);

      DMatrixRMaj JkMinv = new DMatrixRMaj(Jk.getNumRows(), Minv.getNumCols());
      CommonOps_DDRM.mult(Jk, Minv, JkMinv);

      DMatrixRMaj Qk = new DMatrixRMaj(Jk.getNumRows(), C.getNumCols());
      CommonOps_DDRM.mult(JkMinv, C, Qk);
      CommonOps_DDRM.subtractEquals(Qk, JkDot);
      return Qk;
   }

   /**
    * <pre>
    * C<sub>k</sub> = M<sub>k</sub>Q<sub>k</sub>T<sub>k</sub><sup>T</sup>
    * </pre>
    * 
    * Equation (11)
    */
   private DMatrixRMaj computeTaskSpaceCoriolisAndCentrifugalMatrix(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      DMatrixRMaj Mk = computeEndEffectorInertiaMatrix(endEffector, controlFrame);
      DMatrixRMaj Qk = computeQk(endEffector, controlFrame);
      DMatrixRMaj Tk = computeDynamicallyConsistentEndEffectorJacobianInverse(endEffector, controlFrame);
      DMatrixRMaj MkQk = new DMatrixRMaj(Mk.getNumRows(), Qk.getNumCols());
      CommonOps_DDRM.mult(Mk, Qk, MkQk);

      DMatrixRMaj Ck = new DMatrixRMaj(Mk.getNumRows(), Tk.getNumRows());
      CommonOps_DDRM.multTransB(MkQk, Tk, Ck);
      return Ck;
   }

   private DMatrixRMaj computeSystemCoriolisAndCentrifugalMatrix()
   {
      JointBasics[] joints = MultiBodySystemTools.collectSubtreeJoints(rootBody);
      DMatrixRMaj initialJointVelocities = new DMatrixRMaj(numberOfDoFs, 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, initialJointVelocities);

      DMatrixRMaj coriolisMatrix = new DMatrixRMaj(numberOfDoFs, numberOfDoFs);
      DMatrixRMaj jointVelocitiesUnitary = new DMatrixRMaj(numberOfDoFs, 1);

      for (int i = 0; i < numberOfDoFs; i++)
      {
         jointVelocitiesUnitary.zero();
         jointVelocitiesUnitary.set(i, 0, 1.0);
         MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, jointVelocitiesUnitary);
         rootBody.updateFramesRecursively();
         coriolisCalculator.compute();
         CommonOps_DDRM.insert(coriolisCalculator.getJointTauMatrix(), coriolisMatrix, 0, i);
      }

      MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, initialJointVelocities);
      rootBody.updateFramesRecursively();

      return coriolisMatrix;
   }

   /**
    * <pre>
    * T<sub>k</sub> = M<sub>k</sub>J<sub>k</sub>M<sup>-1</sup>
    * </pre>
    * 
    * Equation (12)
    */
   private DMatrixRMaj computeDynamicallyConsistentEndEffectorJacobianInverse(RigidBodyBasics endEffector, ReferenceFrame controlFrame)
   {
      DMatrixRMaj Mk = computeEndEffectorInertiaMatrix(endEffector, controlFrame);
      DMatrixRMaj Jk = computeEndEffectorJacobian(endEffector, controlFrame);
      DMatrixRMaj Minv = computeSystemInertiaMatrixInverse();
      DMatrixRMaj MkJk = new DMatrixRMaj(Mk.getNumRows(), Jk.getNumCols());
      CommonOps_DDRM.mult(Mk, Jk, MkJk);
      DMatrixRMaj Tk = new DMatrixRMaj(Mk.getNumRows(), Minv.getNumCols());
      CommonOps_DDRM.mult(MkJk, Minv, Tk);
      return Tk;
   }
}
