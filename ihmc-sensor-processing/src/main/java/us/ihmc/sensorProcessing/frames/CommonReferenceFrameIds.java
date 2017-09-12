package us.ihmc.sensorProcessing.frames;

public enum CommonReferenceFrameIds
{
   MIDFEET_ZUP_FRAME(-100), PELVIS_ZUP_FRAME(-101), PELVIS_FRAME(-102), CHEST_FRAME(-103), CENTER_OF_MASS_FRAME(-104), LEFT_SOLE_FRAME(-105), RIGHT_SOLE_FRAME(-106);
   
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
