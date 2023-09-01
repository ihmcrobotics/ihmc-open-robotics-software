package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message represents a rigid body transform as a translation and rotation matrix
       */
public class RigidBodyTransformMessage extends Packet<RigidBodyTransformMessage> implements Settable<RigidBodyTransformMessage>, EpsilonComparable<RigidBodyTransformMessage>
{
   /**
            * Translation X
            */
   public double x_;
   /**
            * Translation Y
            */
   public double y_;
   /**
            * Translation Z
            */
   public double z_;
   /**
            * Rotation matrix
            */
   public double m00_;
   /**
            * Rotation matrix
            */
   public double m01_;
   /**
            * Rotation matrix
            */
   public double m02_;
   /**
            * Rotation matrix
            */
   public double m10_;
   /**
            * Rotation matrix
            */
   public double m11_;
   /**
            * Rotation matrix
            */
   public double m12_;
   /**
            * Rotation matrix
            */
   public double m20_;
   /**
            * Rotation matrix
            */
   public double m21_;
   /**
            * Rotation matrix
            */
   public double m22_;

   public RigidBodyTransformMessage()
   {
   }

   public RigidBodyTransformMessage(RigidBodyTransformMessage other)
   {
      this();
      set(other);
   }

   public void set(RigidBodyTransformMessage other)
   {
      x_ = other.x_;

      y_ = other.y_;

      z_ = other.z_;

      m00_ = other.m00_;

      m01_ = other.m01_;

      m02_ = other.m02_;

      m10_ = other.m10_;

      m11_ = other.m11_;

      m12_ = other.m12_;

      m20_ = other.m20_;

      m21_ = other.m21_;

      m22_ = other.m22_;

   }

   /**
            * Translation X
            */
   public void setX(double x)
   {
      x_ = x;
   }
   /**
            * Translation X
            */
   public double getX()
   {
      return x_;
   }

   /**
            * Translation Y
            */
   public void setY(double y)
   {
      y_ = y;
   }
   /**
            * Translation Y
            */
   public double getY()
   {
      return y_;
   }

   /**
            * Translation Z
            */
   public void setZ(double z)
   {
      z_ = z;
   }
   /**
            * Translation Z
            */
   public double getZ()
   {
      return z_;
   }

   /**
            * Rotation matrix
            */
   public void setM00(double m00)
   {
      m00_ = m00;
   }
   /**
            * Rotation matrix
            */
   public double getM00()
   {
      return m00_;
   }

   /**
            * Rotation matrix
            */
   public void setM01(double m01)
   {
      m01_ = m01;
   }
   /**
            * Rotation matrix
            */
   public double getM01()
   {
      return m01_;
   }

   /**
            * Rotation matrix
            */
   public void setM02(double m02)
   {
      m02_ = m02;
   }
   /**
            * Rotation matrix
            */
   public double getM02()
   {
      return m02_;
   }

   /**
            * Rotation matrix
            */
   public void setM10(double m10)
   {
      m10_ = m10;
   }
   /**
            * Rotation matrix
            */
   public double getM10()
   {
      return m10_;
   }

   /**
            * Rotation matrix
            */
   public void setM11(double m11)
   {
      m11_ = m11;
   }
   /**
            * Rotation matrix
            */
   public double getM11()
   {
      return m11_;
   }

   /**
            * Rotation matrix
            */
   public void setM12(double m12)
   {
      m12_ = m12;
   }
   /**
            * Rotation matrix
            */
   public double getM12()
   {
      return m12_;
   }

   /**
            * Rotation matrix
            */
   public void setM20(double m20)
   {
      m20_ = m20;
   }
   /**
            * Rotation matrix
            */
   public double getM20()
   {
      return m20_;
   }

   /**
            * Rotation matrix
            */
   public void setM21(double m21)
   {
      m21_ = m21;
   }
   /**
            * Rotation matrix
            */
   public double getM21()
   {
      return m21_;
   }

   /**
            * Rotation matrix
            */
   public void setM22(double m22)
   {
      m22_ = m22;
   }
   /**
            * Rotation matrix
            */
   public double getM22()
   {
      return m22_;
   }


   public static Supplier<RigidBodyTransformMessagePubSubType> getPubSubType()
   {
      return RigidBodyTransformMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RigidBodyTransformMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RigidBodyTransformMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_, other.x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_, other.y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.z_, other.z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m00_, other.m00_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m01_, other.m01_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m02_, other.m02_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m10_, other.m10_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m11_, other.m11_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m12_, other.m12_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m20_, other.m20_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m21_, other.m21_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.m22_, other.m22_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RigidBodyTransformMessage)) return false;

      RigidBodyTransformMessage otherMyClass = (RigidBodyTransformMessage) other;

      if(this.x_ != otherMyClass.x_) return false;

      if(this.y_ != otherMyClass.y_) return false;

      if(this.z_ != otherMyClass.z_) return false;

      if(this.m00_ != otherMyClass.m00_) return false;

      if(this.m01_ != otherMyClass.m01_) return false;

      if(this.m02_ != otherMyClass.m02_) return false;

      if(this.m10_ != otherMyClass.m10_) return false;

      if(this.m11_ != otherMyClass.m11_) return false;

      if(this.m12_ != otherMyClass.m12_) return false;

      if(this.m20_ != otherMyClass.m20_) return false;

      if(this.m21_ != otherMyClass.m21_) return false;

      if(this.m22_ != otherMyClass.m22_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RigidBodyTransformMessage {");
      builder.append("x=");
      builder.append(this.x_);      builder.append(", ");
      builder.append("y=");
      builder.append(this.y_);      builder.append(", ");
      builder.append("z=");
      builder.append(this.z_);      builder.append(", ");
      builder.append("m00=");
      builder.append(this.m00_);      builder.append(", ");
      builder.append("m01=");
      builder.append(this.m01_);      builder.append(", ");
      builder.append("m02=");
      builder.append(this.m02_);      builder.append(", ");
      builder.append("m10=");
      builder.append(this.m10_);      builder.append(", ");
      builder.append("m11=");
      builder.append(this.m11_);      builder.append(", ");
      builder.append("m12=");
      builder.append(this.m12_);      builder.append(", ");
      builder.append("m20=");
      builder.append(this.m20_);      builder.append(", ");
      builder.append("m21=");
      builder.append(this.m21_);      builder.append(", ");
      builder.append("m22=");
      builder.append(this.m22_);
      builder.append("}");
      return builder.toString();
   }
}
