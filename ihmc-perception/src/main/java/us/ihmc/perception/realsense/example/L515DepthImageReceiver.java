package us.ihmc.perception.realsense.example;

import java.nio.ShortBuffer;

import org.bytedeco.librealsense2.rs2_frame;

public interface L515DepthImageReceiver
{
   /**
    * This callback gets called from the L515 update method after we get a depth image
    * @param depthFrame - the raw rs2_frame that can be used with the realsense2 library
    * @param depthData - the depth data as unsigned ints. This is a 1D array that represents a 2D depth image.
    * 
    * Example (Note the Explicit call to deal with unsigned numbers. Java doesn't like them):
    * 
    * int index = y * l515.getDepthWidth() + x;
    * int val = Short.toUnsignedInt(depthData.get(index));
    * double distance = val * l515.getDepthToMeterConversion();
    */
   public void receivedDepthData(rs2_frame depthFrame, ShortBuffer depthData);
}
