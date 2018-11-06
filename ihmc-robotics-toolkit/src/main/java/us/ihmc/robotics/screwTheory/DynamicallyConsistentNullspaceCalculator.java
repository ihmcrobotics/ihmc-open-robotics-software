package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

/**
 * @author twan
 *         Date: 4/17/13
 */
public interface DynamicallyConsistentNullspaceCalculator
{
   void reset();

   void addConstraint(RigidBody body, DenseMatrix64F selectionMatrix);

   void addActuatedJoint(JointBasics joint);

   void compute();

   DenseMatrix64F getDynamicallyConsistentNullspace();

   DenseMatrix64F getSNsBar();
}
