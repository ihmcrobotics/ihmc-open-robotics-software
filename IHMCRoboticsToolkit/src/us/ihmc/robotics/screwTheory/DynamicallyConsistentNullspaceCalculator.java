package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

/**
 * @author twan
 *         Date: 4/17/13
 */
public interface DynamicallyConsistentNullspaceCalculator
{
   void reset();

   void addConstraint(RigidBody body, DenseMatrix64F selectionMatrix);

   void addActuatedJoint(InverseDynamicsJoint joint);

   void compute();

   DenseMatrix64F getDynamicallyConsistentNullspace();

   DenseMatrix64F getSNsBar();
}
