package us.ihmc.footstepPlanning.swing;

import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.sensors.FootSwitchFactory;

// TODO think about extracting high level controller info relevant to the planner somewhere so they can be logged, added to ui, etc.
// SwingOverPlanarRegions directly reconstructs controller trajectory, so it might be a bad idea to duplicate some parameters (e.g. desired touchdown velocity)
// and try to keep them synced. But reusing controller parameter classes doesn't seem great either

// As an intermediate step, low-hanging fruit would be just doing the foot polygons and replacing the foot geometry parameters (e.g. foot forward offset)
// from the controller param classes with foot polygons

public class ProxyAtlasWalkingControllerParameters extends WalkingControllerParameters
{
   @Override
   public double getOmega0()
   {
      return 0;
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return false;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      return 0;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return 0;
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return false;
   }

   @Override
   public PDGains getCoMHeightControlGains()
   {
      return null;
   }

   @Override
   public PIDSE3Configuration getSwingFootControlGains()
   {
      return null;
   }

   @Override
   public PIDSE3Configuration getHoldPositionFootControlGains()
   {
      return null;
   }

   @Override
   public PIDSE3Configuration getToeOffFootControlGains()
   {
      return null;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return 0;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return 0;
   }

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      return null;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      return new String[0];
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return null;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportForwardX()
   {
      return 0;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportInnerY()
   {
      return 0;
   }

   @Override
   public StepAdjustmentParameters getStepAdjustmentParameters()
   {
      return null;
   }

   @Override
   public ToeOffParameters getToeOffParameters()
   {
      return null;
   }

   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return new ProxyAtlasSwingTrajectoryParameters();
   }

   @Override
   public ICPControllerParameters getICPControllerParameters()
   {
      return null;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return 0;
   }

   @Override
   public double minimumHeightAboveAnkle()
   {
      return 0;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return 0;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return 0;
   }

   @Override
   public SteppingParameters getSteppingParameters()
   {
      return new ProxyAtlasSteppingParameters();
   }

   private static class ProxyAtlasSwingTrajectoryParameters extends SwingTrajectoryParameters
   {
      @Override
      public double getMinSwingHeight()
      {
         return 0.10;
      }

      @Override
      public double getDefaultSwingHeight()
      {
         return getMinSwingHeight();
      }

      @Override
      public double getMaxSwingHeight()
      {
         return 0.30;
      }

      @Override
      public double getDesiredTouchdownHeightOffset()
      {
         return 0;
      }

      @Override
      public double getDesiredTouchdownVelocity()
      {
         return -0.3;
      }

      @Override
      public double getDesiredTouchdownAcceleration()
      {
         return -1.0;
      }

      /**
       * {@inheritDoc}
       */
      @Override
      public boolean addOrientationMidpointForObstacleClearance()
      {
         return false;
      }

      /**
       * {@inheritDoc}
       */
      @Override
      public boolean useSingularityAvoidanceInSupport()
      {
         return true;
      }
   }

   private static class ProxyAtlasSteppingParameters implements SteppingParameters
   {
      @Override
      public double getFootForwardOffset()
      {
         return getFootLength() - getFootBackwardOffset();
      }

      @Override
      public double getFootBackwardOffset()
      {
         return 0.085;
      }

      @Override
      public double getInPlaceWidth()
      {
         return 0.25;
      }

      @Override
      public double getMaxStepLength()
      {
         return 0.6; // 0.5; //0.35;
      }

      @Override
      public double getMinStepWidth()
      {
         return 0.15;
      }

      @Override
      public double getMaxStepWidth()
      {
         return 0.6; // 0.4;
      }


      @Override
      public double getDefaultStepLength()
      {
         return 0.6;
      }

      @Override
      public double getMaxStepUp()
      {
         return 0.25;
      }

      @Override
      public double getMaxStepDown()
      {
         return 0.2;
      }

      @Override
      public double getMaxAngleTurnOutwards()
      {
         //increased atlas turn speed defaults
         // return Math.PI / 4.0;
         return 0.6;
      }

      @Override
      public double getMaxAngleTurnInwards()
      {
         //increased atlas turn speed defaults
         //  return 0;
         return -0.1;
      }

      @Override
      public double getTurningStepWidth()
      {
         return 0.25;
      }

      @Override
      public double getFootWidth()
      {
         return 0.11;
      }

      @Override
      public double getToeWidth()
      {
         return 0.085;
      }

      @Override
      public double getFootLength()
      {
         return 0.22;
      }

      @Override
      public double getActualFootWidth()
      {
         return 0.138;
      }

      @Override
      public double getActualFootLength()
      {
         return 0.26;
      }
   }

   public static ConvexPolygon2D getProxyAtlasFootPolygon()
   {
      SteppingParameters steppingParameters = new ProxyAtlasSteppingParameters();

      ConvexPolygon2D foot = new ConvexPolygon2D();
      foot.addVertex(steppingParameters.getFootForwardOffset(), -0.5 * steppingParameters.getToeWidth());
      foot.addVertex(steppingParameters.getFootForwardOffset(), 0.5 * steppingParameters.getToeWidth());
      foot.addVertex(-steppingParameters.getFootBackwardOffset(), -0.5 * steppingParameters.getFootWidth());
      foot.addVertex(-steppingParameters.getFootBackwardOffset(), 0.5 * steppingParameters.getFootWidth());
      foot.update();

      return foot;
   }
}
