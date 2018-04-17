package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class Video extends Packet<Video> implements Settable<Video>, EpsilonComparable<Video>
{
   public boolean hasTimebase_;

   public Video()
   {
   }

   public Video(Video other)
   {
      this();
      set(other);
   }

   public void set(Video other)
   {
      hasTimebase_ = other.hasTimebase_;

   }

   public void setHasTimebase(boolean hasTimebase)
   {
      hasTimebase_ = hasTimebase;
   }
   public boolean getHasTimebase()
   {
      return hasTimebase_;
   }


   @Override
   public boolean epsilonEquals(Video other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hasTimebase_, other.hasTimebase_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Video)) return false;

      Video otherMyClass = (Video) other;

      if(this.hasTimebase_ != otherMyClass.hasTimebase_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Video {");
      builder.append("hasTimebase=");
      builder.append(this.hasTimebase_);
      builder.append("}");
      return builder.toString();
   }
}
