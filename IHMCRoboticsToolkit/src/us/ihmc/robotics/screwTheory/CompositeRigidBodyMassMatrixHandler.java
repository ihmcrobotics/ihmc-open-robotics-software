package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import java.util.ArrayList;

public class CompositeRigidBodyMassMatrixHandler
{
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;

   public CompositeRigidBodyMassMatrixHandler(RigidBody rootBody, ArrayList<InverseDynamicsJoint> jointsToIgnore)
   {
      this.massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody, jointsToIgnore);
   }

   public void compute()
   {
      massMatrixCalculator.compute();
   }

   public void getMassMatrix(InverseDynamicsJoint[] jointsToConsider, DenseMatrix64F massMatrixToPack)
   {
      massMatrixCalculator.getMassMatrix(jointsToConsider, massMatrixToPack);
   }
}
