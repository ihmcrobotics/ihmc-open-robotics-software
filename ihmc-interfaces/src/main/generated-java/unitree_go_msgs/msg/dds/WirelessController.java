package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WirelessController extends Packet<WirelessController> implements Settable<WirelessController>, EpsilonComparable<WirelessController>
{
   public float lx_;
   public float ly_;
   public float rx_;
   public float ry_;
   public int keys_;

   public WirelessController()
   {
   }

   public WirelessController(WirelessController other)
   {
      this();
      set(other);
   }

   public void set(WirelessController other)
   {
      lx_ = other.lx_;

      ly_ = other.ly_;

      rx_ = other.rx_;

      ry_ = other.ry_;

      keys_ = other.keys_;

   }

   public void setLx(float lx)
   {
      lx_ = lx;
   }
   public float getLx()
   {
      return lx_;
   }

   public void setLy(float ly)
   {
      ly_ = ly;
   }
   public float getLy()
   {
      return ly_;
   }

   public void setRx(float rx)
   {
      rx_ = rx;
   }
   public float getRx()
   {
      return rx_;
   }

   public void setRy(float ry)
   {
      ry_ = ry;
   }
   public float getRy()
   {
      return ry_;
   }

   public void setKeys(int keys)
   {
      keys_ = keys;
   }
   public int getKeys()
   {
      return keys_;
   }


   public static Supplier<WirelessControllerPubSubType> getPubSubType()
   {
      return WirelessControllerPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WirelessControllerPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WirelessController other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lx_, other.lx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ly_, other.ly_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rx_, other.rx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ry_, other.ry_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.keys_, other.keys_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WirelessController)) return false;

      WirelessController otherMyClass = (WirelessController) other;

      if(this.lx_ != otherMyClass.lx_) return false;

      if(this.ly_ != otherMyClass.ly_) return false;

      if(this.rx_ != otherMyClass.rx_) return false;

      if(this.ry_ != otherMyClass.ry_) return false;

      if(this.keys_ != otherMyClass.keys_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WirelessController {");
      builder.append("lx=");
      builder.append(this.lx_);      builder.append(", ");
      builder.append("ly=");
      builder.append(this.ly_);      builder.append(", ");
      builder.append("rx=");
      builder.append(this.rx_);      builder.append(", ");
      builder.append("ry=");
      builder.append(this.ry_);      builder.append(", ");
      builder.append("keys=");
      builder.append(this.keys_);
      builder.append("}");
      return builder.toString();
   }
}
