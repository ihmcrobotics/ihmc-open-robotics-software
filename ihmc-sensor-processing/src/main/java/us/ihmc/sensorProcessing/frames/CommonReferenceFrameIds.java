package us.ihmc.sensorProcessing.frames;

public enum CommonReferenceFrameIds
{
   NONE(-1),
   MIDFEET_ZUP_FRAME(-100),
   PELVIS_ZUP_FRAME(-101),
   MIDFEET_ZUP_GROUND_FRAME(-102),
   WALKING_FRAME(-103),
   PELVIS_FRAME(-104),
   CHEST_FRAME(-105),
   CENTER_OF_MASS_FRAME(-106),
   LEFT_SOLE_FRAME(-107),
   RIGHT_SOLE_FRAME(-108);

   private final long hashId;

   CommonReferenceFrameIds(long hashId)
   {
      this.hashId = hashId;
   }

   public long getHashId()
   {
      return hashId;
   }
}
