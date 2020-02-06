package us.ihmc.commonWalkingControlModules.configurations;

import java.util.EnumSet;

public enum CoPPointName
{
   MIDFEET_COP, ENTRY_COP, MIDFOOT_COP, EXIT_COP, START_COP, FINAL_COP, FLAMINGO_STANCE_FINAL_COP, TOE_COP;

   public static final EnumSet<CoPPointName> set = EnumSet.allOf(CoPPointName.class);
   public static final CoPPointName[] values = values();

   public String toString()
   {
      switch (this)
      {
      case ENTRY_COP:
         return "EntryCoP";
      case MIDFOOT_COP:
         return "MidfootCoP";
      case EXIT_COP:
         return "ExitCoP";
      case TOE_COP:
         return "ToeCoP";
      case MIDFEET_COP:
         return "MidFeetCoP";
      case FINAL_COP:
         return "EndCoP";
      case FLAMINGO_STANCE_FINAL_COP:
         return "FlamingoStanceFinalCoP";
      default:
         return "StartCoP";
      }
   }
}

