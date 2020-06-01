package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC robot environment awareness module (REA).
       * This provides access to a few filters used in REA.
       */
public class REASensorDataFilterParametersMessage extends Packet<REASensorDataFilterParametersMessage> implements Settable<REASensorDataFilterParametersMessage>, EpsilonComparable<REASensorDataFilterParametersMessage>
{

   /**
            * Lower-bound of the bounding box inside which sensor data will be processed into planar regions.
            * Note that the coordinates are relative to the sensor position and yaw:
            * - x-coordinate: represents the distance in front (positive) or in the back (negative) of the sensor.
            * - y-coordinate: represents the distance to the left (positive) or to the right (negative) of the sensor.
            * - z-coordinate: represents the distance above (positive) or below (negative) of the sensor.
            * Sensor data collected outside this bounding box is still used to help estimating empty space inside it.
            * If any of the lower-bound coordinates is NaN, the bounding box parameters will be ignored.
            */
   public us.ihmc.euclid.tuple3D.Point3D bounding_box_min_;

   /**
            * Upper-bound of the bounding box inside which sensor data will be processed into planar regions.
            * Note that the coordinates are relative to the sensor position and yaw:
            * - x-coordinate: represents the distance in front (positive) or in the back (negative) of the sensor.
            * - y-coordinate: represents the distance to the left (positive) or to the right (negative) of the sensor.
            * - z-coordinate: represents the distance above (positive) or below (negative) of the sensor.
            * Sensor data collected outside this bounding box is still used to help estimating empty space inside it.
            * If any of the upper-bound coordinates is NaN, the bounding box parameters will be ignored.
            */
   public us.ihmc.euclid.tuple3D.Point3D bounding_box_max_;

   /**
            * Defines a bounding sphere centered at the sensor within which sensor data will be processed into planar regions.
            * Sensor data collected outside this bounding sphere is still used to help estimating empty space inside it.
            * A negative value will be ignored.
            */
   public double sensor_max_range_;

   /**
            * Defines a bounding sphere centered at the sensor within which sensor data is to be ignored.
            * A negative value will be ignored.
            */
   public double sensor_min_range_;

   public REASensorDataFilterParametersMessage()
   {

      bounding_box_min_ = new us.ihmc.euclid.tuple3D.Point3D();

      bounding_box_max_ = new us.ihmc.euclid.tuple3D.Point3D();



   }

   public REASensorDataFilterParametersMessage(REASensorDataFilterParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(REASensorDataFilterParametersMessage other)
   {

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bounding_box_min_, bounding_box_min_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bounding_box_max_, bounding_box_max_);

      sensor_max_range_ = other.sensor_max_range_;


      sensor_min_range_ = other.sensor_min_range_;

   }



   /**
            * Lower-bound of the bounding box inside which sensor data will be processed into planar regions.
            * Note that the coordinates are relative to the sensor position and yaw:
            * - x-coordinate: represents the distance in front (positive) or in the back (negative) of the sensor.
            * - y-coordinate: represents the distance to the left (positive) or to the right (negative) of the sensor.
            * - z-coordinate: represents the distance above (positive) or below (negative) of the sensor.
            * Sensor data collected outside this bounding box is still used to help estimating empty space inside it.
            * If any of the lower-bound coordinates is NaN, the bounding box parameters will be ignored.
            */
   public us.ihmc.euclid.tuple3D.Point3D getBoundingBoxMin()
   {
      return bounding_box_min_;
   }



   /**
            * Upper-bound of the bounding box inside which sensor data will be processed into planar regions.
            * Note that the coordinates are relative to the sensor position and yaw:
            * - x-coordinate: represents the distance in front (positive) or in the back (negative) of the sensor.
            * - y-coordinate: represents the distance to the left (positive) or to the right (negative) of the sensor.
            * - z-coordinate: represents the distance above (positive) or below (negative) of the sensor.
            * Sensor data collected outside this bounding box is still used to help estimating empty space inside it.
            * If any of the upper-bound coordinates is NaN, the bounding box parameters will be ignored.
            */
   public us.ihmc.euclid.tuple3D.Point3D getBoundingBoxMax()
   {
      return bounding_box_max_;
   }


   /**
            * Defines a bounding sphere centered at the sensor within which sensor data will be processed into planar regions.
            * Sensor data collected outside this bounding sphere is still used to help estimating empty space inside it.
            * A negative value will be ignored.
            */
   public void setSensorMaxRange(double sensor_max_range)
   {
      sensor_max_range_ = sensor_max_range;
   }
   /**
            * Defines a bounding sphere centered at the sensor within which sensor data will be processed into planar regions.
            * Sensor data collected outside this bounding sphere is still used to help estimating empty space inside it.
            * A negative value will be ignored.
            */
   public double getSensorMaxRange()
   {
      return sensor_max_range_;
   }


   /**
            * Defines a bounding sphere centered at the sensor within which sensor data is to be ignored.
            * A negative value will be ignored.
            */
   public void setSensorMinRange(double sensor_min_range)
   {
      sensor_min_range_ = sensor_min_range;
   }
   /**
            * Defines a bounding sphere centered at the sensor within which sensor data is to be ignored.
            * A negative value will be ignored.
            */
   public double getSensorMinRange()
   {
      return sensor_min_range_;
   }


   public static Supplier<REASensorDataFilterParametersMessagePubSubType> getPubSubType()
   {
      return REASensorDataFilterParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return REASensorDataFilterParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(REASensorDataFilterParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!this.bounding_box_min_.epsilonEquals(other.bounding_box_min_, epsilon)) return false;

      if (!this.bounding_box_max_.epsilonEquals(other.bounding_box_max_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sensor_max_range_, other.sensor_max_range_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sensor_min_range_, other.sensor_min_range_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof REASensorDataFilterParametersMessage)) return false;

      REASensorDataFilterParametersMessage otherMyClass = (REASensorDataFilterParametersMessage) other;


      if (!this.bounding_box_min_.equals(otherMyClass.bounding_box_min_)) return false;

      if (!this.bounding_box_max_.equals(otherMyClass.bounding_box_max_)) return false;

      if(this.sensor_max_range_ != otherMyClass.sensor_max_range_) return false;


      if(this.sensor_min_range_ != otherMyClass.sensor_min_range_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("REASensorDataFilterParametersMessage {");

      builder.append("bounding_box_min=");
      builder.append(this.bounding_box_min_);      builder.append(", ");

      builder.append("bounding_box_max=");
      builder.append(this.bounding_box_max_);      builder.append(", ");

      builder.append("sensor_max_range=");
      builder.append(this.sensor_max_range_);      builder.append(", ");

      builder.append("sensor_min_range=");
      builder.append(this.sensor_min_range_);
      builder.append("}");
      return builder.toString();
   }
}
