package us.ihmc.robotics.trajectories;


public class Waypoint1D implements Comparable<Waypoint1D>
{
   static public final double DONT_CARE          = -0.123456789;
   static public final double SURPRISE_ME        = DONT_CARE;
   static public final double I_AM_FEELING_LUCKY = DONT_CARE;
   
   public double absTime;
   public double position;
   public double velocity;
   public double acceleration;
   
   public Waypoint1D()
   {
      absTime  = 0;
      position = DONT_CARE;
      velocity = DONT_CARE;
      acceleration = DONT_CARE;
   }
   
   public Waypoint1D(double time, double pos, double vel, double acc)
   {
      absTime  = time;
      position = pos;
      velocity = vel;
      acceleration = acc;
   }

   
   public Waypoint1D(double pos)
   {
      absTime  = DONT_CARE;
      position = pos;
      velocity = DONT_CARE;
      acceleration = DONT_CARE;
   }
   public Waypoint1D(Waypoint1D other)
   {
      absTime  = other.absTime;
      position = other.position;
      velocity = other.velocity;
      acceleration = other.acceleration;
   }
   @Override
   public int compareTo(Waypoint1D other)
   {
      if (this.absTime > other.absTime) {
         return 1;
      } else if (this.absTime == other.absTime) {
         return 0;
      } else {
         return -1;
      }
   }
}