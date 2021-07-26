package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;

import java.util.function.Supplier;

public class EnvironmentInitialSetup
{
   private final Supplier<PlanarRegionsList> planarRegionsSupplier;
   private final CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface;
   private final double groundZ;
   private final double initialYaw;
   private final double initialX;
   private final double initialY;
   private Point3D initialPosition;

   public EnvironmentInitialSetup(Supplier<PlanarRegionsList> planarRegionsSupplier,
                                  double groundZ,
                                  double initialYaw,
                                  double initialX,
                                  double initialY)
   {
      this(planarRegionsSupplier, null, groundZ, initialYaw, initialX, initialY);
   }

   public EnvironmentInitialSetup(Supplier<PlanarRegionsList> planarRegionsSupplier,
                                  CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface,
                                  double groundZ,
                                  double initialYaw,
                                  double initialX,
                                  double initialY)
   {
      this.planarRegionsSupplier = planarRegionsSupplier;
      this.commonAvatarEnvironmentInterface = commonAvatarEnvironmentInterface;
      this.groundZ = groundZ;
      this.initialYaw = initialYaw;
      this.initialX = initialX;
      this.initialY = initialY;

      initialPosition = new Point3D(initialX, initialY, groundZ);
   }

   public Supplier<PlanarRegionsList> getPlanarRegionsSupplier()
   {
      return planarRegionsSupplier;
   }

   public boolean hasCommonAvatarEnvironmentInterface()
   {
      return commonAvatarEnvironmentInterface != null;
   }

   public CommonAvatarEnvironmentInterface getCommonAvatarEnvironmentInterface()
   {
      return commonAvatarEnvironmentInterface;
   }

   public Point3D getInitialPosition()
   {
      return initialPosition;
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
