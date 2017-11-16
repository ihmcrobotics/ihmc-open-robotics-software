package us.ihmc.commonWalkingControlModules.configurations;

import java.util.EnumSet;

import us.ihmc.robotics.robotSide.RobotSide;

public enum CoPPointName
{
   MIDFEET_COP, HEEL_COP, BALL_COP, TOE_COP, START_COP, FINAL_COP, FLAMINGO_STANCE_FINAL_COP;

   public static final EnumSet<CoPPointName> set = EnumSet.allOf(CoPPointName.class);
   public static final CoPPointName[] values = values();

   public String toString()
   {
      switch (this)
      {
      case HEEL_COP:
         return "HeelCoP";
      case BALL_COP:
         return "BallCoP";
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

   public boolean checkCoPPointMatch(CoPPointName other)
   {
      return this == other;
   }
}

