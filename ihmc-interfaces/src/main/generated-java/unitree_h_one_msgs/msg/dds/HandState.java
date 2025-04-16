package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandState extends Packet<HandState> implements Settable<HandState>, EpsilonComparable<HandState>
{
   public us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.MotorState>  motor_state_;
   public unitree_h_one_msgs.msg.dds.IMUState imu_state_;
   public us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.PressSensorState>  press_sensor_state_;
   public float power_v_;
   public float power_a_;

   public HandState()
   {
      motor_state_ = new us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.MotorState> (100, new unitree_h_one_msgs.msg.dds.MotorStatePubSubType());
      imu_state_ = new unitree_h_one_msgs.msg.dds.IMUState();
      press_sensor_state_ = new us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.PressSensorState> (100, new unitree_h_one_msgs.msg.dds.PressSensorStatePubSubType());

   }

   public HandState(HandState other)
   {
      this();
      set(other);
   }

   public void set(HandState other)
   {
      motor_state_.set(other.motor_state_);
      unitree_h_one_msgs.msg.dds.IMUStatePubSubType.staticCopy(other.imu_state_, imu_state_);
      press_sensor_state_.set(other.press_sensor_state_);
      power_v_ = other.power_v_;

      power_a_ = other.power_a_;

   }


   public us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.MotorState>  getMotorState()
   {
      return motor_state_;
   }


   public unitree_h_one_msgs.msg.dds.IMUState getImuState()
   {
      return imu_state_;
   }


   public us.ihmc.idl.IDLSequence.Object<unitree_h_one_msgs.msg.dds.PressSensorState>  getPressSensorState()
   {
      return press_sensor_state_;
   }

   public void setPowerV(float power_v)
   {
      power_v_ = power_v;
   }
   public float getPowerV()
   {
      return power_v_;
   }

   public void setPowerA(float power_a)
   {
      power_a_ = power_a;
   }
   public float getPowerA()
   {
      return power_a_;
   }


   public static Supplier<HandStatePubSubType> getPubSubType()
   {
      return HandStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.motor_state_.size() != other.motor_state_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.motor_state_.size(); i++)
         {  if (!this.motor_state_.get(i).epsilonEquals(other.motor_state_.get(i), epsilon)) return false; }
      }

      if (!this.imu_state_.epsilonEquals(other.imu_state_, epsilon)) return false;
      if (this.press_sensor_state_.size() != other.press_sensor_state_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.press_sensor_state_.size(); i++)
         {  if (!this.press_sensor_state_.get(i).epsilonEquals(other.press_sensor_state_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.power_v_, other.power_v_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.power_a_, other.power_a_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandState)) return false;

      HandState otherMyClass = (HandState) other;

      if (!this.motor_state_.equals(otherMyClass.motor_state_)) return false;
      if (!this.imu_state_.equals(otherMyClass.imu_state_)) return false;
      if (!this.press_sensor_state_.equals(otherMyClass.press_sensor_state_)) return false;
      if(this.power_v_ != otherMyClass.power_v_) return false;

      if(this.power_a_ != otherMyClass.power_a_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandState {");
      builder.append("motor_state=");
      builder.append(this.motor_state_);      builder.append(", ");
      builder.append("imu_state=");
      builder.append(this.imu_state_);      builder.append(", ");
      builder.append("press_sensor_state=");
      builder.append(this.press_sensor_state_);      builder.append(", ");
      builder.append("power_v=");
      builder.append(this.power_v_);      builder.append(", ");
      builder.append("power_a=");
      builder.append(this.power_a_);
      builder.append("}");
      return builder.toString();
   }
}
