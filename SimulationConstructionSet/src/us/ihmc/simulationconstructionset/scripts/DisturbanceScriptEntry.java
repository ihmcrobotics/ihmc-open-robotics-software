package us.ihmc.simulationconstructionset.scripts;

import javax.vecmath.Vector3d;

public class DisturbanceScriptEntry implements Comparable<Object>
{

   private final double time;
   private final Vector3d forceVector = new Vector3d();
   private final double duration;

   public DisturbanceScriptEntry(double time, double magnitude, Vector3d direction, double duration)
   {
      this(time, direction, duration);
      forceVector.normalize();
      forceVector.scale(magnitude);
   }

   public DisturbanceScriptEntry(double time, Vector3d forceVector, double duration)
   {
      this.time = time;
      this.forceVector.set(forceVector);
      this.duration = duration;
   }

   @Override
   public int compareTo(Object disturbanceScriptEntry)
   {
      if (disturbanceScriptEntry == this)
         return 0;
      if (((DisturbanceScriptEntry) disturbanceScriptEntry).time < time)
         return 1;
      else
         return -1;
   }

   public double getTime()
   {
      return this.time;
   }

   public Vector3d getForceVector()
   {
      return forceVector;
   }

   public double getDuration()
   {
      return duration;
   }
}
