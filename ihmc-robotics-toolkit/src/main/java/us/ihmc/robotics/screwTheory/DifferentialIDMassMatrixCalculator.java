package us.ihmc.robotics.screwTheory;

import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 * Very inefficient. Should only be used for verification of better methods in unit tests
 */
public class DifferentialIDMassMatrixCalculator implements MassMatrixCalculator
{
   private final InverseDynamicsCalculator idCalculator;
   private final JointBasics[] jointsInOrder;
   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F storedJointDesiredAccelerations;
   private final DenseMatrix64F tmpDesiredJointAccelerationsMatrix;
   private final DenseMatrix64F tmpTauMatrix;
   private final LinkedHashMap<JointBasics, Wrench> storedJointWrenches = new LinkedHashMap<JointBasics, Wrench>();
   private final DenseMatrix64F storedJointVelocities;
   
   private final int totalNumberOfDoFs;

   public DifferentialIDMassMatrixCalculator(ReferenceFrame inertialFrame, RigidBodyBasics rootBody)
   {
      SpatialAcceleration zeroRootAcceleration = ScrewTools.createGravitationalSpatialAcceleration(rootBody, 0.0);
      
      idCalculator = new InverseDynamicsCalculator(rootBody, false, true);
      idCalculator.setRootAcceleration(zeroRootAcceleration);
      jointsInOrder = MultiBodySystemTools.collectSubtreeJoints(rootBody);
      totalNumberOfDoFs = MultiBodySystemTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrix = new DenseMatrix64F(totalNumberOfDoFs, totalNumberOfDoFs);
      
      storedJointDesiredAccelerations = new DenseMatrix64F(totalNumberOfDoFs, 1);
      storedJointVelocities = new DenseMatrix64F(totalNumberOfDoFs, 1);
      tmpDesiredJointAccelerationsMatrix = new DenseMatrix64F(totalNumberOfDoFs, 1);
      tmpTauMatrix = new DenseMatrix64F(totalNumberOfDoFs, 1);
      
      for (JointBasics joint : jointsInOrder)
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
         MultiBodySystemTools.insertJointsState(jointsInOrder, JointStateType.ACCELERATION, tmpDesiredJointAccelerationsMatrix);
         
         idCalculator.compute();
         tmpTauMatrix.set(idCalculator.getJointTauMatrix());
         MatrixTools.setMatrixBlock(massMatrix, 0, column, tmpTauMatrix, 0, 0, totalNumberOfDoFs, 1, 1.0);
         column++;
         
         tmpDesiredJointAccelerationsMatrix.set(i, 0, 0.0);
      }
      
      restoreJointState();
   }

   private void setDesiredAccelerationsToZero()
   {
      for (JointBasics joint : jointsInOrder)
      {
         joint.setJointAccelerationToZero();
         joint.setJointVelocity(0, new DenseMatrix64F(joint.getDegreesOfFreedom(), 1));
      }
   }

   private void storeJointState()
   {
      MultiBodySystemTools.extractJointsState(jointsInOrder, JointStateType.ACCELERATION, storedJointDesiredAccelerations);
      MultiBodySystemTools.extractJointsState(jointsInOrder, JointStateType.VELOCITY, storedJointVelocities);
      for (JointBasics joint : jointsInOrder)
      {
         DenseMatrix64F tauMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         joint.getJointTau(0, tauMatrix);
         DenseMatrix64F spatialForce = new DenseMatrix64F(SpatialForce.SIZE, 1);
         DenseMatrix64F motionSubspace = new DenseMatrix64F(6, joint.getDegreesOfFreedom());
         joint.getMotionSubspace(motionSubspace);
         CommonOps.mult(motionSubspace, tauMatrix, spatialForce);
         Wrench jointWrench = storedJointWrenches.get(joint);
         jointWrench.setIncludingFrame(joint.getFrameAfterJoint(), spatialForce);
         jointWrench.changeFrame(joint.getFrameAfterJoint());
      }
   }
   
   private void restoreJointState()
   {
      MultiBodySystemTools.insertJointsState(jointsInOrder, JointStateType.ACCELERATION, storedJointDesiredAccelerations);
      MultiBodySystemTools.insertJointsState(jointsInOrder, JointStateType.VELOCITY, storedJointVelocities);
      
      for (JointBasics joint : jointsInOrder)
      {
         joint.setJointWrench(storedJointWrenches.get(joint));
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
   public JointBasics[] getJointsInOrder()
   {
      return jointsInOrder;
   }
}
