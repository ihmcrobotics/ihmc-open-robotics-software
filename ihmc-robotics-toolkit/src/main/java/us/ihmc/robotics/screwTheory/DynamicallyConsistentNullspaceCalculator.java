package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

/**
 * @author twan
 *         Date: 4/17/13
 */
public interface DynamicallyConsistentNullspaceCalculator
{
   void reset();

   void addConstraint(RigidBodyBasics body, DenseMatrix64F selectionMatrix);

   void addActuatedJoint(JointBasics joint);

   void compute();

   DenseMatrix64F getDynamicallyConsistentNullspace();

   DenseMatrix64F getSNsBar();
}
