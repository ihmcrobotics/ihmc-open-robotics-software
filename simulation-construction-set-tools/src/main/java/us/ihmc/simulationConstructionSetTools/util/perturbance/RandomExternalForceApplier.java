package us.ihmc.simulationConstructionSetTools.util.perturbance;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RandomExternalForceApplier implements RobotController
{
   private final YoRegistry registry = new YoRegistry("PushApplier");
   private String name;

   private final ForcePerturbable perturbable;
   private final YoDouble maximalDisturbanceMagnitude;
   private final YoFrameVector3D currentDisturbanceForce;

   private final Random random = new Random(1776L);

   public RandomExternalForceApplier(ForcePerturbable perturbable, double maximalDisturbanceMagnitude, String name)
   {
      this.perturbable = perturbable;
      this.name = name;
      this.maximalDisturbanceMagnitude = new YoDouble("maximalDisturbanceMagnitude", registry);
      this.maximalDisturbanceMagnitude.set(maximalDisturbanceMagnitude);
      this.currentDisturbanceForce = new YoFrameVector3D("currentDisturbanceForce", "", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public void doControl()
   {
      computeCurrentDisturbanceForce();
      perturbable.setForcePerturbance(currentDisturbanceForce, Double.POSITIVE_INFINITY);
   }

   public void computeCurrentDisturbanceForce()
   {
      currentDisturbanceForce.setX(random.nextDouble());
      currentDisturbanceForce.setY(random.nextDouble());
      currentDisturbanceForce.setZ(random.nextDouble());
      currentDisturbanceForce.normalize();
      currentDisturbanceForce.scale(maximalDisturbanceMagnitude.getDoubleValue());
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return this.name;
   }
   
   @Override
   public void initialize()
   {      
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
