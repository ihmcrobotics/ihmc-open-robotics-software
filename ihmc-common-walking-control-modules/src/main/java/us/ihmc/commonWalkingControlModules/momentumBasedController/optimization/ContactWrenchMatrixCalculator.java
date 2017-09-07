package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ContactWrenchMatrixCalculator
{
   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final JointIndexHandler jointIndexHandler;
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private final DenseMatrix64F contactableBodyJacobianMatrix = new DenseMatrix64F(6, 12);

   private final RigidBody rootBody;

   private final int numberOfDoFs;

   private final DenseMatrix64F tmpFullContactJacobianMatrix;
   private final DenseMatrix64F tmpContactJacobianMatrixTranspose;
   private final DenseMatrix64F tmpContactJacobianMatrix;

   public ContactWrenchMatrixCalculator(RigidBody rootBody, List<? extends ContactablePlaneBody> contactablePlaneBodies, WrenchMatrixCalculator wrenchMatrixCalculator,
         JointIndexHandler jointIndexHandler)
   {
      this.rootBody = rootBody;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;
      this.jointIndexHandler = jointIndexHandler;

      numberOfDoFs = jointIndexHandler.getNumberOfDoFs();
      int rhoSize = wrenchMatrixCalculator.getRhoSize();

      tmpFullContactJacobianMatrix = new DenseMatrix64F(rhoSize, numberOfDoFs);
      tmpContactJacobianMatrixTranspose = new DenseMatrix64F(numberOfDoFs, rhoSize);
      tmpContactJacobianMatrix = new DenseMatrix64F(rhoSize, numberOfDoFs);
   }

   public void computeContactForceJacobian(DenseMatrix64F contactForceJacobianToPack)
   {
      int contactForceStartIndex = 0;
      for (int bodyIndex = 0; bodyIndex < contactablePlaneBodies.size(); bodyIndex++)
      {
         RigidBody rigidBody = contactablePlaneBodies.get(bodyIndex).getRigidBody();
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(rootBody, rigidBody);
         jacobianCalculator.setJacobianFrame(wrenchMatrixCalculator.getJacobianFrame());
         jacobianCalculator.computeJacobianMatrix();
         jacobianCalculator.getJacobianMatrix(contactableBodyJacobianMatrix);

         DenseMatrix64F rhoJacobianMatrix = wrenchMatrixCalculator.getRhoJacobianMatrix(rigidBody);

         int rhoSize = rhoJacobianMatrix.getNumCols();

         tmpContactJacobianMatrixTranspose.reshape(contactableBodyJacobianMatrix.getNumCols(), rhoSize);
         tmpContactJacobianMatrix.reshape(rhoSize, contactableBodyJacobianMatrix.getNumCols());
         CommonOps.multTransA(contactableBodyJacobianMatrix, rhoJacobianMatrix, tmpContactJacobianMatrixTranspose);
         CommonOps.transpose(tmpContactJacobianMatrixTranspose, tmpContactJacobianMatrix);

         jointIndexHandler.compactBlockToFullBlock(jacobianCalculator.getJointsFromBaseToEndEffector(), tmpContactJacobianMatrix, tmpFullContactJacobianMatrix);
         CommonOps.extract(tmpFullContactJacobianMatrix, 0, rhoSize, 0, numberOfDoFs, contactForceJacobianToPack, contactForceStartIndex, 0);

         contactForceStartIndex += rhoSize;
      }
   }
}
