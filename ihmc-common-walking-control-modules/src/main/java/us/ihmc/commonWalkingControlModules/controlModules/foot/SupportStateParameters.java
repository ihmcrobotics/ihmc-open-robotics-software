package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SupportStateParameters
{
   private static final double defaultFootLoadThreshold = 0.2;

   // For testing:
   private final BooleanProvider assumeCopOnEdge;
   private final BooleanProvider assumeFootBarelyLoaded;
   private final BooleanProvider neverHoldRotation;
   private final BooleanProvider neverHoldPosition;

   // For line contact walking and balancing:
   private final BooleanProvider holdFootOrientationFlat;

   // Toe contact point loading time
   private final boolean rampUpAllowableToeLoadAfterContact;
   private final YoDouble toeLoadingDuration;
   private final YoDouble fullyLoadedMagnitude;

   private final YoDouble footLoadThreshold;

   public SupportStateParameters(WalkingControllerParameters walkingControllerParameters, YoRegistry parentRegistry)
   {
      String prefix = "Foot";
      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());


      assumeCopOnEdge = new BooleanParameter(prefix + "AssumeCopOnEdge", registry, false);
      assumeFootBarelyLoaded = new BooleanParameter(prefix + "AssumeFootBarelyLoaded", registry, false);
      neverHoldRotation = new BooleanParameter(prefix + "NeverHoldRotation", registry, false);
      neverHoldPosition = new BooleanParameter(prefix + "NeverHoldPosition", registry, false);
      holdFootOrientationFlat = new BooleanParameter(prefix + "HoldFlatOrientation", registry, false);

      rampUpAllowableToeLoadAfterContact = walkingControllerParameters.rampUpAllowableToeLoadAfterContact();
      toeLoadingDuration = new YoDouble(prefix + "ToeContactPointLoadingTime", registry);
      fullyLoadedMagnitude = new YoDouble(prefix + "FullyLoadedMagnitude", registry);
      toeLoadingDuration.set(walkingControllerParameters.getToeLoadingDuration());
      fullyLoadedMagnitude.set(walkingControllerParameters.getFullyLoadedToeForce());

      footLoadThreshold = new YoDouble(prefix + "LoadThreshold", registry);
      footLoadThreshold.set(defaultFootLoadThreshold);

      parentRegistry.addChild(registry);
   }

   public boolean assumeCopOnEdge()
   {
      return assumeCopOnEdge.getValue();
   }

   public boolean assumeFootBarelyLoaded()
   {
      return assumeFootBarelyLoaded.getValue();
   }

   public boolean neverHoldRotation()
   {
      return neverHoldRotation.getValue();
   }

   public boolean neverHoldPosition()
   {
      return neverHoldPosition.getValue();
   }

   public boolean holdFootOrientationFlat()
   {
      return holdFootOrientationFlat.getValue();
   }

   public boolean rampUpAllowableToeLoadAfterContact()
   {
      return rampUpAllowableToeLoadAfterContact;
   }

   public double getToeLoadingDuration()
   {
      return toeLoadingDuration.getDoubleValue();
   }

   public double getFullyLoadedMagnitude()
   {
      return fullyLoadedMagnitude.getDoubleValue();
   }

   public double getFootLoadThreshold()
   {
      return footLoadThreshold.getDoubleValue();
   }
}
