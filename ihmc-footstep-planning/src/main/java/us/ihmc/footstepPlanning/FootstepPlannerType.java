package us.ihmc.footstepPlanning;

public enum FootstepPlannerType
{
   PLANAR_REGION_BIPEDAL, PLAN_THEN_SNAP, A_STAR, SIMPLE_BODY_PATH, VIS_GRAPH_WITH_A_STAR;

   public static final FootstepPlannerType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlannerType fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
