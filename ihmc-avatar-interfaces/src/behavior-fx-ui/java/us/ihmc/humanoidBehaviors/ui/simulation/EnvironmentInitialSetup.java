package us.ihmc.humanoidBehaviors.ui.simulation;

import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.function.Supplier;

public class EnvironmentInitialSetup
{
   private final Supplier<PlanarRegionsList> planarRegionsSupplier;
   private final double groundZ;
   private final double initialYaw;
   private final double initialX;
   private final double initialY;

   public EnvironmentInitialSetup(Supplier<PlanarRegionsList> planarRegionsSupplier,
                                  double groundZ,
                                  double initialYaw,
                                  double initialX,
                                  double initialY)
   {
      this.planarRegionsSupplier = planarRegionsSupplier;
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
      this.initialX = initialX;
      this.initialY = initialY;
   }

   public Supplier<PlanarRegionsList> getPlanarRegionsSupplier()
   {
      return planarRegionsSupplier;
   }

   public double getGroundZ()
   {
      return groundZ;
   }

   public double getInitialYaw()
   {
      return initialYaw;
   }

   public double getInitialX()
   {
      return initialX;
   }

   public double getInitialY()
   {
      return initialY;
   }
}
