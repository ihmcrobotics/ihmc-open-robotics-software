package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface WholeBodyContactStateInterface
{
   /**
    * Number of contact points
    */
   int getNumberOfContactPoints();

   /**
    * Each contact frame has an origin at the contact point and the z-axis aligned with the environment surface normal (e.g. z is upward for flat ground)
    */
   ReferenceFrame getContactFrame(int contactPointIndex);

   /**
    * Coefficient of friction for the given contact point
    */
   double getCoefficientOfFriction(int contactPointIndex);

   /**
    * The actuation constraint matrix is C in the constraint C f <= d
    * Where f = [f_0x, f_0y, f_0z, f_1x... ] are the ground reaction forces in world frame.
    */
   DMatrixRMaj getActuationConstraintMatrix();

   /**
    * The actuation constraint vector is d in the constraint C f <= d
    * Where f = [f_0x, f_0y, f_0z, f_1x... ] are the ground reaction forces in world frame.
    */
   DMatrixRMaj getActuationConstraintVector();
}
