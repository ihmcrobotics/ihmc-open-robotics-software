package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

import java.util.List;

public class ContactWrenchMatrixCalculator
{
   private final WrenchMatrixCalculator wrenchMatrixCalculator;
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final JointIndexHandler jointIndexHandler;
   private final GeometricJacobianHolder geometricJacobianHolder;

   private final RigidBody rootBody;

   private final int numberOfDoFs;

   private final DenseMatrix64F tmpFullContactJacobianMatrix;
   private final DenseMatrix64F tmpContactJacobianMatrixTranspose;
   private final DenseMatrix64F tmpContactJacobianMatrix;

   public ContactWrenchMatrixCalculator(RigidBody rootBody, List<? extends ContactablePlaneBody> contactablePlaneBodies, WrenchMatrixCalculator wrenchMatrixCalculator,
         JointIndexHandler jointIndexHandler, GeometricJacobianHolder geometricJacobianHolder)
   {
      this.rootBody = rootBody;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.wrenchMatrixCalculator = wrenchMatrixCalculator;
      this.jointIndexHandler = jointIndexHandler;
      this.geometricJacobianHolder = geometricJacobianHolder;

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
         long jacobianID = geometricJacobianHolder.getOrCreateGeometricJacobian(rootBody, rigidBody, wrenchMatrixCalculator.getJacobianFrame());
         GeometricJacobian geometricJacobian = geometricJacobianHolder.getJacobian(jacobianID);

         DenseMatrix64F contactableBodyJacobianMatrix = geometricJacobian.getJacobianMatrix();
         DenseMatrix64F rhoJacobianMatrix = wrenchMatrixCalculator.getRhoJacobianMatrix(rigidBody);

         int rhoSize = rhoJacobianMatrix.getNumCols();

         tmpContactJacobianMatrixTranspose.reshape(contactableBodyJacobianMatrix.getNumCols(), rhoSize);
         tmpContactJacobianMatrix.reshape(rhoSize, contactableBodyJacobianMatrix.getNumCols());
         CommonOps.multTransA(contactableBodyJacobianMatrix, rhoJacobianMatrix, tmpContactJacobianMatrixTranspose);
         CommonOps.transpose(tmpContactJacobianMatrixTranspose, tmpContactJacobianMatrix);

         jointIndexHandler.compactBlockToFullBlock(geometricJacobian.getJointsInOrder(), tmpContactJacobianMatrix, tmpFullContactJacobianMatrix);
         CommonOps.extract(tmpFullContactJacobianMatrix, 0, rhoSize, 0, numberOfDoFs, contactForceJacobianToPack, contactForceStartIndex, 0);

         contactForceStartIndex += rhoSize;
      }
   }
}
