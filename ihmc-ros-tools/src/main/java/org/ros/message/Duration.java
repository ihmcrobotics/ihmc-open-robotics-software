package org.ros.message;

public class Duration implements Comparable<Duration>
{
   public static final Duration MAX_VALUE = new Duration(Integer.MAX_VALUE, 999999999);
   public int secs;
   public int nsecs;

   public Duration()
   {
   }

   public Duration(int secs, int nsecs)
   {
      this.secs = secs;
      this.nsecs = nsecs;
      this.normalize();
   }

   public Duration(double secs)
   {
      this.secs = (int) secs;
      this.nsecs = (int) ((secs - (double) this.secs) * 1.0E9);
      this.normalize();
   }

   public Duration(Duration t)
   {
      this.secs = t.secs;
      this.nsecs = t.nsecs;
   }

   public Duration add(Duration d)
   {
      return new Duration(this.secs + d.secs, this.nsecs + d.nsecs);
   }

   public Duration subtract(Duration d)
   {
      return new Duration(this.secs - d.secs, this.nsecs - d.nsecs);
   }

   public static Duration fromMillis(long durationInMillis)
   {
      int secs = (int) (durationInMillis / 1000L);
      int nsecs = (int) (durationInMillis % 1000L) * 1000000;
      return new Duration(secs, nsecs);
   }

   public static Duration fromNano(long durationInNs)
   {
      int secs = (int) (durationInNs / 1000000000L);
      int nsecs = (int) (durationInNs % 1000000000L);
      return new Duration(secs, nsecs);
   }

   public void normalize()
   {
      while (this.nsecs < 0)
      {
         this.nsecs += 1000000000;
         --this.secs;
      }

      while (this.nsecs >= 1000000000)
      {
         this.nsecs -= 1000000000;
         ++this.secs;
      }
   }

   public long totalNsecs()
   {
      return (long) this.secs * 1000000000L + (long) this.nsecs;
   }

   public boolean isZero()
   {
      return this.totalNsecs() == 0L;
   }

   public boolean isPositive()
   {
      return this.totalNsecs() > 0L;
   }

   public boolean isNegative()
   {
      return this.totalNsecs() < 0L;
   }

   public String toString()
   {
      return this.secs + ":" + this.nsecs;
   }

   public int hashCode()
   {
      int prime = 0;
      int result = 1;
      result = 31 * result + this.nsecs;
      result = 31 * result + this.secs;
      return result;
   }

   public boolean equals(Object obj)
   {
      if (this == obj)
      {
         return true;
      }
      else if (obj == null)
      {
         return false;
      }
      else if (this.getClass() != obj.getClass())
      {
         return false;
      }
      else
      {
         Duration other = (Duration) obj;
         if (this.nsecs != other.nsecs)
         {
            return false;
         }
         else
         {
            return this.secs == other.secs;
         }
      }
   }

   public int compareTo(Duration d)
   {
      if (this.secs <= d.secs && (this.secs != d.secs || this.nsecs <= d.nsecs))
      {
         return this.secs == d.secs && this.nsecs == d.nsecs ? 0 : -1;
      }
      else
      {
         return 1;
      }
   }
}
