package us.ihmc.simulationConstructionSetTools.util.environments;

public enum ValveType
{
   SMALL_VALVE, BIG_VALVE, CUSTOM;

   public double getValveRadius()
   {
      switch (this)
      {
      case SMALL_VALVE:
         return 0.14;
      case BIG_VALVE:
         return 0.20;
      case CUSTOM:
         return 0.15;
      default:
         return 0.0;
      }
   }
   
   public double getValveThickness()
   {
      switch (this)
      {
      case SMALL_VALVE:
         return 0.025;
      case BIG_VALVE:
         return 0.038;
      case CUSTOM:
         return 0.05;
      default:
         return 0.0;
      }
   }
   
   public double getValveMass()
   {
      switch (this)
      {
      case SMALL_VALVE:
         return 2.0;
      case BIG_VALVE:
         return 3.0;
      case CUSTOM:
         return 3.0;
      default:
         return 0.0;
      }
   }
   
   public double getValveOffsetFromWall()
   {
      switch (this)
      {
      case SMALL_VALVE:
         return 0.18;
      case BIG_VALVE:
         return 0.33;
      case CUSTOM:
         return 0.40;
      default:
         return 0.0;
      }
   }

   public int getNumberOfSpokes()
   {
      switch (this)
      {
      case SMALL_VALVE:
         return 4;
      case BIG_VALVE:
         return 5;
      case CUSTOM:
         return 3;
      default:
         return 0;
      }
   }
   
   public double getSpokesThickness()
   {
      switch (this)
      {
      case SMALL_VALVE:
         return 0.025;
      case BIG_VALVE:
         return 0.038;
      case CUSTOM:
         return 0.040;
      default:
         return 0.0;
      }
   }
}
