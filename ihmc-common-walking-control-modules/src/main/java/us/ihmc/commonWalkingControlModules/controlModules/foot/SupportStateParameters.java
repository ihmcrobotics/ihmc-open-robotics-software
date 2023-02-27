package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SupportStateParameters
{
   private static final double defaultFootLoadThreshold = 0.2;

   private static final double EPSILON_POINT_ON_EDGE = 5e-3;
   private static final double EPSILON_POINT_ON_EDGE_WITH_HYSTERESIS = 8e-3;

   // For testing:
   private final BooleanProvider assumeCopOnEdge;
   private final BooleanProvider assumeFootBarelyLoaded;
   private final BooleanProvider neverHoldRotation;
   private final BooleanProvider neverHoldPosition;

   // For line contact walking and balancing:
   private final BooleanProvider holdFootOrientationFlat;

   // Toe contact point loading time
   private final YoDouble footLoadThreshold;
   private final DoubleProvider copOnEdgeEpsilon;
   private final DoubleProvider copOnEdgeEpsilonWithHysteresis;


   public SupportStateParameters(WalkingControllerParameters walkingControllerParameters, YoRegistry parentRegistry)
   {
      String prefix = "Foot";
      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());

      copOnEdgeEpsilon = new DoubleParameter(prefix + "CopOnEdgeEpsilon", registry, EPSILON_POINT_ON_EDGE);
      copOnEdgeEpsilonWithHysteresis = new DoubleParameter(prefix + "CopOnEdgeEpsilonWithHysteresis", registry, EPSILON_POINT_ON_EDGE_WITH_HYSTERESIS);

      assumeCopOnEdge = new BooleanParameter(prefix + "AssumeCopOnEdge", registry, false);
      assumeFootBarelyLoaded = new BooleanParameter(prefix + "AssumeFootBarelyLoaded", registry, false);
      neverHoldRotation = new BooleanParameter(prefix + "NeverHoldRotation", registry, false);
      neverHoldPosition = new BooleanParameter(prefix + "NeverHoldPosition", registry, false);
      holdFootOrientationFlat = new BooleanParameter(prefix + "HoldFlatOrientation", registry, false);

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

   public double getFootLoadThreshold()
   {
      return footLoadThreshold.getDoubleValue();
   }

   public double getCopOnEdgeEpsilon()
   {
      return copOnEdgeEpsilon.getValue();
   }

   public double getCopOnEdgeEpsilonWithHysteresis()
   {
      return copOnEdgeEpsilonWithHysteresis.getValue();
   }
}
