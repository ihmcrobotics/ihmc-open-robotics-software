package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

/**
 * This time, some additional parameters have to be defined.
 */
public class RaymanOptimizationSettings implements ControllerCoreOptimizationSettings
{
   /** {@inheritDoc} */
   // @Override
   // public double getJointVelocityWeight()
   //
   // return ControllerCoreOptimizationSettings.super.getJointVelocityWeight();
   //

   /** {@inheritDoc} */
   @Override
   public double getJointAccelerationWeight()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getJointJerkWeight()
   {
      return 0.0001;
   }

   @Override
   public double getRhoWeight()
   { // This refers to the regularization of the contact force variables in the
     // optimization.
      return 0.01;
   }

   @Override
   public double getRhoMin()
   {
      /*
       * rho min is used to define the unilaterality of the contact forces. Ideally it should be equal to
       * zero, but making slightly greater helps to prevent the feet from rotating when in support.
       */
      return 0.5;
   }

   @Override
   public int getNumberOfContactableBodies()
   {
      // Refers to the maximum number of contactable bodies.
      return 2;
   }

   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      // Refers to the maximum number of contact points per contactable body.
      return 4;
   }

   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      /*
       * The basis vectors are used to approximate the friction cone for each contact point. so the more
       * the better, however increasing this number also drastically increases the optimization size.
       * Three basis vectors is the minimum and four represents a good tradeoff between computation and
       * accuracy.
       */
      return 4;
   }

   @Override
   public double getRhoRateDefaultWeight()
   {
      return 0;
   }

   @Override
   public Vector2D getCoPWeight()
   {
      return new Vector2D();
   }

   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return new Vector2D();
   }
}
