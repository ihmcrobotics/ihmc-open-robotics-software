package us.ihmc.zulu.parameters.controller;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class ZuluSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   @Override
   public double getFinalCoMVelocityInjectionRatio()
   {
      return 0.5;
   }

   @Override
   public double getFinalCoMAccelerationInjectionRatio()
   {
      return 0.0;
   }

   @Override
   public boolean addOrientationMidpointForObstacleClearance()
   {
      return true;
   }

   @Override
   public Tuple3DReadOnly getTouchdownVelocityWeight()
   {
      return new Vector3D(30.0, 30.0, Double.POSITIVE_INFINITY);
   }

   @Override
   public double getDefaultSwingHeight()
   {
      return 0.1;
   }

   @Override
   public double getMinSwingHeight()
   {
      return 0.025;
   }

   @Override
   public double getMaxSwingHeight()
   {
      // TODO Needs tune up.
      return 0.35;
   }

   @Override
   public boolean useInitialToeHeight()
   {
      return true;
   }

   @Override
   public boolean useFinalHeelHeight()
   {
      return true;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0.0; // TODO Tune me up!
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      // TODO Needs tune up
      return -0.15;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      // TODO Needs tune up
      return -2.0;
   }

   @Override
   public double getBlindFootstepsHeightOffset()
   {
      return 0.0;
   }

   @Override
   public double getDefaultSwingStepUpHeight()
   {
      return 0.13;
   }

   @Override
   public double getDefaultSwingStepDownHeight()
   {
      return 0.10;
   }

   @Override
   public double getMinHeightDifferenceForStepUpOrDown()
   {
      return 0.05;
   }

   @Override
   public double[] getSwingStepUpWaypointProportions()
   {
      return new double[] {0.05, 0.80};
   }

   @Override
   public double[] getSwingStepDownWaypointProportions()
   {
      return new double[] {0.20, 0.95};
   }

   @Override
   public double getFirstWaypointHeightFactorForSteppingUp()
   {
      return 1.5 / 3.0;
   }

   @Override
   public double getSecondWaypointHeightFactorForSteppingDown()
   {
      return 0.5;
   }

   /** {@inheritDoc} **/
   @Override
   public boolean addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown()
   {
      return true;
   }
}