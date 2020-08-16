package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WorkspaceLimiterParameters
{
   private static final double defaultMinVelocityDifference = 5e-4;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble percentOfLegLengthMarginToDisableSingularityAvoidance;
   private final YoDouble percentOfLegLengthMarginToAbortSingularityAvoidance;
   private final YoDouble percentOfLegLengthMarginToEnableSingularityAvoidance;
   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot;
   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight;
   private final YoDouble minPercentOfLegLengthForSingularityAvoidanceInSwing;

   private final YoDouble maxPercentOfLegLengthForSingularityAvoidanceInSupport;

   private final YoDouble velocityDifferenceForLengthening;
   private final YoDouble timeToCorrectForUnachievedSwingTranslation;
   private final YoDouble correctionAlphaFilter;
   private final YoDouble alphaUnreachableFootstep;

   public WorkspaceLimiterParameters(YoRegistry parentRegistry)
   {
      String namePrefix = "foot";
      alphaUnreachableFootstep = new YoDouble(namePrefix + "AlphaUnreachableFootstep", registry);
      alphaUnreachableFootstep.set(0.75);

      correctionAlphaFilter = new YoDouble(namePrefix + "CorrectionAlphaFilter", registry);
      correctionAlphaFilter.set(0.98);

      timeToCorrectForUnachievedSwingTranslation = new YoDouble(namePrefix + "TimeToCorrectForUnachievedSwingTranslation", registry);
      timeToCorrectForUnachievedSwingTranslation.set(0.2);

      maxPercentOfLegLengthForSingularityAvoidanceInSupport = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSupport", registry);
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSwingForFoot", registry);
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight = new YoDouble(namePrefix + "MaxPercOfLegLengthForSingularityAvoidanceInSwingForHeight", registry);
      minPercentOfLegLengthForSingularityAvoidanceInSwing = new YoDouble(namePrefix + "MinPercOfLegLengthForSingularityAvoidanceInSwing", registry);
      percentOfLegLengthMarginToEnableSingularityAvoidance = new YoDouble(namePrefix + "PercMarginToEnableSingularityAvoidance", registry);
      percentOfLegLengthMarginToDisableSingularityAvoidance = new YoDouble(namePrefix + "PercMarginToDisableSingularityAvoidance", registry);
      percentOfLegLengthMarginToAbortSingularityAvoidance = new YoDouble(namePrefix + "PercMarginToAbortSingularityAvoidance", registry);

      maxPercentOfLegLengthForSingularityAvoidanceInSupport.set(0.98);
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForFoot.set(0.97);
      maxPercentOfLegLengthForSingularityAvoidanceInSwingForHeight.set(0.95);
      minPercentOfLegLengthForSingularityAvoidanceInSwing.set(0.5);

      percentOfLegLengthMarginToEnableSingularityAvoidance.set(0.1);
      percentOfLegLengthMarginToDisableSingularityAvoidance.set(0.12);
      percentOfLegLengthMarginToAbortSingularityAvoidance.set(0.17);

      velocityDifferenceForLengthening = new YoDouble(namePrefix + "VelocityDifferenceForLengthening", registry);
      velocityDifferenceForLengthening.set(defaultMinVelocityDifference);

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

   public DoubleProvider getPercentOfLegLengthMarginToEnableSingularityAvoidance()
   {
      return percentOfLegLengthMarginToEnableSingularityAvoidance;
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
