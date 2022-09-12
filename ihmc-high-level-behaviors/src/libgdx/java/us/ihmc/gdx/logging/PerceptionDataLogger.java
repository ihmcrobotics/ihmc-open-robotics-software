package us.ihmc.gdx.logging;

import org.bytedeco.hdf5.H5File;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.RealtimeROS2Node;

import static org.bytedeco.hdf5.global.hdf5.H5F_ACC_TRUNC;

public class PerceptionDataLogger {
    static final int PCD_POINT_SIZE = 3;
    static final String FILE_NAME = "/home/bmishra/Workspace/Data/Sensor_Logs/HDF5/OusterL515Log.h5";
    private H5File file;

    public PerceptionDataLogger(ROS2Node ros2Node) {

//        new IHMCROS2Callback<>(ros2Node, ROS2Tools.MAPSENSE_REGIONS, this::regionsCallback);


        file = new H5File(FILE_NAME, H5F_ACC_TRUNC);
    }

    public void depthCallback()
    {

    }

    public static void main(String[] args) {

        RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");
        ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_data_logger");
//        PerceptionDataLogger logger = new PerceptionDataLogger(realtimeROS2Node);
    }
}


