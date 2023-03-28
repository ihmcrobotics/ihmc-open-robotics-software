package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WorkspaceLimiterParameters
{
   private static final double defaultMinVelocityDifference = 5e-4;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DoubleProvider percentOfLegLengthMarginToDisableSingularityAvoidance;
   private final DoubleProvider percentOfLegLengthMarginToAbortSingularityAvoidance;
   private final DoubleProvider percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot;
   private final DoubleProvider percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight;
   private final DoubleProvider maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot;
   private final DoubleProvider maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight;
   private final DoubleProvider minPercentOfLegLengthForSingularityAvoidanceInSwing;

   private final DoubleProvider maxPercentOfLegLengthForSingularityAvoidanceInSupport;

   private final DoubleProvider velocityDifferenceForLengthening;
   private final DoubleProvider timeToCorrectForUnachievedSwingTranslation;
   private final DoubleProvider correctionAlphaFilter;
   private final DoubleProvider alphaUnreachableFootstep;
   private final BooleanProvider enableSingularityAvoidanceOnSwingFoot;

   public WorkspaceLimiterParameters(YoRegistry parentRegistry)
   {
      String namePrefix = "foot";
      alphaUnreachableFootstep = new DoubleParameter(namePrefix + "AlphaUnreachableFootstep", registry, 0.75);

      correctionAlphaFilter = new DoubleParameter(namePrefix + "CorrectionAlphaFilter", registry, 0.98);

      timeToCorrectForUnachievedSwingTranslation = new DoubleParameter(namePrefix + "TimeToCorrectForUnachievedSwingTranslation", registry, 0.2);

      maxPercentOfLegLengthForSingularityAvoidanceInSupport = new DoubleParameter(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSupport",
                                                                                  registry,
                                                                                  0.98);
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot = new DoubleParameter(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSwingForFoot",
                                                                                       registry,
                                                                                       0.97);
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight = new DoubleParameter(namePrefix
            + "MaxPercOfLegLengthForSingularityAvoidanceInSwingForHeight", registry, 0.94);
      minPercentOfLegLengthForSingularityAvoidanceInSwing = new DoubleParameter(namePrefix + "MinPercOfLegLengthForSingularityAvoidanceInSwing", registry, 0.5);
      percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot = new DoubleParameter(namePrefix + "PercMarginToEnableSingularityAvoidanceForFoot",
                                                                                        registry,
                                                                                        0.05);
      percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight = new DoubleParameter(namePrefix + "PercMarginToEnableSingularityAvoidanceForHeight",
                                                                                          registry,
                                                                                          0.1);
      percentOfLegLengthMarginToDisableSingularityAvoidance = new DoubleParameter(namePrefix + "PercMarginToDisableSingularityAvoidance", registry, 0.12);
      percentOfLegLengthMarginToAbortSingularityAvoidance = new DoubleParameter(namePrefix + "PercMarginToAbortSingularityAvoidance", registry, 0.17);

      velocityDifferenceForLengthening = new DoubleParameter(namePrefix + "VelocityDifferenceForLengthening", registry, defaultMinVelocityDifference);

      enableSingularityAvoidanceOnSwingFoot = new BooleanParameter(namePrefix + "enableSingularityAvoidanceOnSwingFoot", registry, true);

      parentRegistry.addChild(registry);
   }

   public DoubleProvider getAlphaUnreachableFootstep()
   {
      return alphaUnreachableFootstep;
   }

   public DoubleProvider getCorrectionAlphaFilter()
   {
      return correctionAlphaFilter;
   }

   public BooleanProvider getEnableSingularityAvoidanceOnSwingFoot()
   {
      return enableSingularityAvoidanceOnSwingFoot;
   }

   public DoubleProvider getTimeToCorrectForUnachievedSwingTranslation()
   {
      return timeToCorrectForUnachievedSwingTranslation;
   }

   public DoubleProvider getMaxPercentOfLegLengthForSingularityAvoidanceInSupport()
   {
      return maxPercentOfLegLengthForSingularityAvoidanceInSupport;
   }

   public DoubleProvider getMaxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot()
   {
      return maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot;
   }

   public DoubleProvider getMaxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight()
   {
      return maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight;
   }

   public DoubleProvider getMinPercentOfLegLengthForSingularityAvoidanceInSwing()
   {
      return minPercentOfLegLengthForSingularityAvoidanceInSwing;
   }

   public DoubleProvider getPercentOfLegLengthMarginToEnableSingularityAvoidanceForFoot()
   {
      return percentOfLegLengthMarginToEnableSingularityAvoidanceForFoot;
   }

   public DoubleProvider getPercentOfLegLengthMarginToEnableSingularityAvoidanceForHeight()
   {
      return percentOfLegLengthMarginToEnableSingularityAvoidanceForHeight;
   }

   public DoubleProvider getPercentOfLegLengthMarginToDisableSingularityAvoidance()
   {
      return percentOfLegLengthMarginToDisableSingularityAvoidance;
   }

   public DoubleProvider getPercentOfLegLengthMarginToAbortSingularityAvoidance()
   {
      return percentOfLegLengthMarginToAbortSingularityAvoidance;
   }

   public DoubleProvider getVelocityDifferenceForLengthening()
   {
      return velocityDifferenceForLengthening;
   }
}
