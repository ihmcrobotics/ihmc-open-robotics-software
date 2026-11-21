package us.ihmc.gr00t;

/** Immutable bridge metadata and default wire dimensions for one GR00T task embodiment. */
public record Gr00tModelConfiguration(String actionLayout,
                                      String layoutId,
                                      int stateSize,
                                      int actionSize,
                                      int defaultActionHorizon,
                                      double defaultActionRateHz,
                                      int defaultImageWidth,
                                      int defaultImageHeight,
                                      int defaultChunkLength,
                                      String defaultLeftImageKey,
                                      String defaultRightImageKey)
{
   public Gr00tModelConfiguration
   {
      if (actionLayout == null || actionLayout.isBlank())
         throw new IllegalArgumentException("actionLayout must not be blank");
      if (layoutId == null || layoutId.isBlank())
         throw new IllegalArgumentException("layoutId must not be blank");
      if (stateSize <= 0 || actionSize <= 0)
         throw new IllegalArgumentException("stateSize and actionSize must be positive");
      if (defaultActionHorizon <= 0 || defaultChunkLength < defaultActionHorizon)
         throw new IllegalArgumentException("defaultChunkLength must be at least the positive action horizon");
      if (!Double.isFinite(defaultActionRateHz) || defaultActionRateHz <= 0.0)
         throw new IllegalArgumentException("defaultActionRateHz must be finite and positive");
      if (defaultImageWidth <= 0 || defaultImageHeight <= 0)
         throw new IllegalArgumentException("default image dimensions must be positive");
      if (defaultLeftImageKey == null || defaultLeftImageKey.isBlank()
          || defaultRightImageKey == null || defaultRightImageKey.isBlank())
         throw new IllegalArgumentException("default image wire keys must not be blank");
   }
}
