package org.ros.message;

public class Time implements Comparable<Time> {
   public int secs;
   public int nsecs;

   public Time() {
      this.secs = 0;
      this.nsecs = 0;
   }

   public Time(int secs, int nsecs) {
      this.secs = secs;
      this.nsecs = nsecs;
      this.normalize();
   }

   public Time(double secs) {
      this.secs = (int)secs;
      this.nsecs = (int)((secs - (double)this.secs) * 1.0E9);
      this.normalize();
   }

   public Time(Time t) {
      this.secs = t.secs;
      this.nsecs = t.nsecs;
   }

   public Time add(Duration d) {
      return new Time(this.secs + d.secs, this.nsecs + d.nsecs);
   }

   public Time subtract(Duration d) {
      return new Time(this.secs - d.secs, this.nsecs - d.nsecs);
   }

   public Duration subtract(Time t) {
      return new Duration(this.secs - t.secs, this.nsecs - t.nsecs);
   }

   public static Time fromMillis(long timeInMillis) {
      int secs = (int)(timeInMillis / 1000L);
      int nsecs = (int)(timeInMillis % 1000L) * 1000000;
      return new Time(secs, nsecs);
   }

   public static Time fromNano(long timeInNs) {
      int secs = (int)(timeInNs / 1000000000L);
      int nsecs = (int)(timeInNs % 1000000000L);
      return new Time(secs, nsecs);
   }

   public String toString() {
      return this.secs + ":" + this.nsecs;
   }

   public double toSeconds() {
      return (double)this.totalNsecs() / 1.0E9;
   }

   public long totalNsecs() {
      return (long)this.secs * 1000000000L + (long)this.nsecs;
   }

   public boolean isZero() {
      return this.totalNsecs() == 0L;
   }

   public void normalize() {
      while(this.nsecs < 0) {
         this.nsecs += 1000000000;
         --this.secs;
      }

      while(this.nsecs >= 1000000000) {
         this.nsecs -= 1000000000;
         ++this.secs;
      }

   }

   public int hashCode() {
      int prime = 0;
      int result = 1;
      result = 31 * result + this.nsecs;
      result = 31 * result + this.secs;
      return result;
   }

   public boolean equals(Object obj) {
      if (this == obj) {
         return true;
      } else if (obj == null) {
         return false;
      } else if (this.getClass() != obj.getClass()) {
         return false;
      } else {
         Time other = (Time)obj;
         if (this.nsecs != other.nsecs) {
            return false;
         } else {
            return this.secs == other.secs;
         }
      }
   }

   public int compareTo(Time t) {
      if (this.secs <= t.secs && (this.secs != t.secs || this.nsecs <= t.nsecs)) {
         return this.secs == t.secs && this.nsecs == t.nsecs ? 0 : -1;
      } else {
         return 1;
      }
   }
}