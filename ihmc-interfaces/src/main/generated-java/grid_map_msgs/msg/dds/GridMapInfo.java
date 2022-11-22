package grid_map_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GridMapInfo extends Packet<GridMapInfo> implements Settable<GridMapInfo>, EpsilonComparable<GridMapInfo>
{
    /**
     * Resolution of the grid [m/cell].
     */
    public double resolution_;
    /**
     * Length in x-direction [m].
     */
    public double length_x_;
    /**
     * Length in y-direction [m].
     */
    public double length_y_;
    /**
     * Pose of the grid map center in the frame defined in `header` [m].
     */
    public us.ihmc.euclid.geometry.Pose3D pose_;

    public GridMapInfo()
    {
        pose_ = new us.ihmc.euclid.geometry.Pose3D();
    }

    public GridMapInfo(GridMapInfo other)
    {
        this();
        set(other);
    }

    public void set(GridMapInfo other)
    {
        resolution_ = other.resolution_;

        length_x_ = other.length_x_;

        length_y_ = other.length_y_;

        geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
    }

    /**
     * Resolution of the grid [m/cell].
     */
    public void setResolution(double resolution)
    {
        resolution_ = resolution;
    }
    /**
     * Resolution of the grid [m/cell].
     */
    public double getResolution()
    {
        return resolution_;
    }

    /**
     * Length in x-direction [m].
     */
    public void setLengthX(double length_x)
    {
        length_x_ = length_x;
    }
    /**
     * Length in x-direction [m].
     */
    public double getLengthX()
    {
        return length_x_;
    }

    /**
     * Length in y-direction [m].
     */
    public void setLengthY(double length_y)
    {
        length_y_ = length_y;
    }
    /**
     * Length in y-direction [m].
     */
    public double getLengthY()
    {
        return length_y_;
    }


    /**
     * Pose of the grid map center in the frame defined in `header` [m].
     */
    public us.ihmc.euclid.geometry.Pose3D getPose()
    {
        return pose_;
    }


    public static Supplier<GridMapInfoPubSubType> getPubSubType()
    {
        return GridMapInfoPubSubType::new;
    }

    @Override
    public Supplier<TopicDataType> getPubSubTypePacket()
    {
        return GridMapInfoPubSubType::new;
    }

    @Override
    public boolean epsilonEquals(GridMapInfo other, double epsilon)
    {
        if(other == null) return false;
        if(other == this) return true;

        if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resolution_, other.resolution_, epsilon)) return false;

        if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.length_x_, other.length_x_, epsilon)) return false;

        if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.length_y_, other.length_y_, epsilon)) return false;

        if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;

        return true;
    }

    @Override
    public boolean equals(Object other)
    {
        if(other == null) return false;
        if(other == this) return true;
        if(!(other instanceof GridMapInfo)) return false;

        GridMapInfo otherMyClass = (GridMapInfo) other;

        if(this.resolution_ != otherMyClass.resolution_) return false;

        if(this.length_x_ != otherMyClass.length_x_) return false;

        if(this.length_y_ != otherMyClass.length_y_) return false;

        if (!this.pose_.equals(otherMyClass.pose_)) return false;

        return true;
    }

    @Override
    public java.lang.String toString()
    {
        StringBuilder builder = new StringBuilder();

        builder.append("GridMapInfo {");
        builder.append("resolution=");
        builder.append(this.resolution_);      builder.append(", ");
        builder.append("length_x=");
        builder.append(this.length_x_);      builder.append(", ");
        builder.append("length_y=");
        builder.append(this.length_y_);      builder.append(", ");
        builder.append("pose=");
        builder.append(this.pose_);
        builder.append("}");
        return builder.toString();
    }
}