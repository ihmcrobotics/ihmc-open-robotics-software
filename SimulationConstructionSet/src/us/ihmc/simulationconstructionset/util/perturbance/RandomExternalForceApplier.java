package us.ihmc.simulationconstructionset.util.perturbance;

import java.util.Random;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class RandomExternalForceApplier implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PushApplier");
   private String name;

   private final ForcePerturbable perturbable;
   private final DoubleYoVariable maximalDisturbanceMagnitude;
   private final YoFrameVector currentDisturbanceForce;

   private final Random random = new Random(1776L);

   public RandomExternalForceApplier(ForcePerturbable perturbable, double maximalDisturbanceMagnitude, String name)
   {
      this.perturbable = perturbable;
      this.name = name;
      this.maximalDisturbanceMagnitude = new DoubleYoVariable("maximalDisturbanceMagnitude", registry);
      this.maximalDisturbanceMagnitude.set(maximalDisturbanceMagnitude);
      this.currentDisturbanceForce = new YoFrameVector("currentDisturbanceForce", "", ReferenceFrame.getWorldFrame(), registry);
   }

   public void doControl()
   {
      computeCurrentDisturbanceForce();
      perturbable.setForcePerturbance(currentDisturbanceForce.getFrameVectorCopy().getVector(), Double.POSITIVE_INFINITY);
   }

   public void computeCurrentDisturbanceForce()
   {
      currentDisturbanceForce.setX(random.nextDouble());
      currentDisturbanceForce.setY(random.nextDouble());
      currentDisturbanceForce.setZ(random.nextDouble());
      currentDisturbanceForce.normalize();
      currentDisturbanceForce.scale(maximalDisturbanceMagnitude.getDoubleValue());
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return this.name;
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }
}
