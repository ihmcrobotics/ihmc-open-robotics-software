package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Very inefficient. Should only be used for verification of better methods in unit tests
 */
public class DifferentialIDMassMatrixCalculator implements MassMatrixCalculator
{
   private final InverseDynamicsCalculator idCalculator;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final DenseMatrix64F massMatrix;
   private final TwistCalculator twistCalculator;
   private final DenseMatrix64F storedJointDesiredAccelerations;
   private final DenseMatrix64F tmpDesiredJointAccelerationsMatrix;
   private final DenseMatrix64F tmpTauMatrix;
   private final LinkedHashMap<InverseDynamicsJoint, Wrench> storedJointWrenches = new LinkedHashMap<InverseDynamicsJoint, Wrench>();
   private final DenseMatrix64F storedJointVelocities;
   
   private final int totalNumberOfDoFs;

   public DifferentialIDMassMatrixCalculator(ReferenceFrame inertialFrame, RigidBody rootBody)
   {
      twistCalculator = new TwistCalculator(inertialFrame, rootBody);
      LinkedHashMap<RigidBody, Wrench> zeroExternalWrench = new LinkedHashMap<RigidBody, Wrench>();
      ArrayList<InverseDynamicsJoint> zeroJointToIgnore = new ArrayList<InverseDynamicsJoint>();
      SpatialAccelerationVector zeroRootAcceleration = ScrewTools.createGravitationalSpatialAcceleration(rootBody, 0.0);
      
      idCalculator = new InverseDynamicsCalculator(inertialFrame, zeroRootAcceleration, zeroExternalWrench, zeroJointToIgnore, false, true, twistCalculator);
      jointsInOrder = ScrewTools.computeSubtreeJoints(rootBody);
      totalNumberOfDoFs = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrix = new DenseMatrix64F(totalNumberOfDoFs, totalNumberOfDoFs);
      
      storedJointDesiredAccelerations = new DenseMatrix64F(totalNumberOfDoFs, 1);
      storedJointVelocities = new DenseMatrix64F(totalNumberOfDoFs, 1);
      tmpDesiredJointAccelerationsMatrix = new DenseMatrix64F(totalNumberOfDoFs, 1);
      tmpTauMatrix = new DenseMatrix64F(totalNumberOfDoFs, 1);
      
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         ReferenceFrame bodyFixedFrame = joint.getSuccessor().getBodyFixedFrame();
         Wrench jointWrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
         storedJointWrenches.put(joint, jointWrench);
      }
   }

   @Override
   public void compute()
   {
      storeJointState();
      setDesiredAccelerationsToZero();

      int column = 0;
      
      for (int i = 0 ; i < totalNumberOfDoFs; i++)
      {
         tmpDesiredJointAccelerationsMatrix.set(i, 0, 1.0);
         ScrewTools.setDesiredAccelerations(jointsInOrder, tmpDesiredJointAccelerationsMatrix);
         
         idCalculator.compute();
         tmpTauMatrix.set(ScrewTools.getTauMatrix(jointsInOrder));
         MatrixTools.setMatrixBlock(massMatrix, 0, column, tmpTauMatrix, 0, 0, totalNumberOfDoFs, 1, 1.0);
         column++;
         
         tmpDesiredJointAccelerationsMatrix.set(i, 0, 0.0);
      }
      
      restoreJointState();
   }

   private void setDesiredAccelerationsToZero()
   {
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         joint.setDesiredAccelerationToZero();
         joint.setVelocity(new DenseMatrix64F(joint.getDegreesOfFreedom(), 1), 0);
      }
   }

   private void storeJointState()
   {
      ScrewTools.getDesiredJointAccelerationsMatrix(jointsInOrder, storedJointDesiredAccelerations);
      ScrewTools.getJointVelocitiesMatrix(jointsInOrder, storedJointVelocities);
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         DenseMatrix64F tauMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         joint.getTauMatrix(tauMatrix);
         DenseMatrix64F spatialForce = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
         CommonOps.mult(joint.getMotionSubspace().getJacobianMatrix(), tauMatrix, spatialForce);
         Wrench jointWrench = storedJointWrenches.get(joint);
         jointWrench.set(joint.getFrameAfterJoint(), spatialForce);
         jointWrench.changeFrame(joint.getSuccessor().getBodyFixedFrame());
      }
   }
   
   private void restoreJointState()
   {
      ScrewTools.setDesiredAccelerations(jointsInOrder, storedJointDesiredAccelerations);
      ScrewTools.setVelocities(jointsInOrder, storedJointVelocities);
      
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         joint.setTorqueFromWrench(storedJointWrenches.get(joint));
      }
   }

   @Override
   public DenseMatrix64F getMassMatrix()
   {
      return massMatrix;
   }

   @Override
   public void getMassMatrix(DenseMatrix64F massMatrixToPack)
   {
      massMatrixToPack.set(massMatrix);
   }

   @Override
   public InverseDynamicsJoint[] getJointsInOrder()
   {
      return jointsInOrder;
   }
}
