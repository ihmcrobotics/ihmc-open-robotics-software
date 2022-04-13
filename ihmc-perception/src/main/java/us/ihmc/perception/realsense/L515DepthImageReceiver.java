package us.ihmc.perception.realsense;

import java.nio.ShortBuffer;

import org.bytedeco.librealsense2.rs2_frame;

public interface L515DepthImageReceiver
{
   /**
    * This callback gets called from the L515 update method after we get a depth image
    * @param depthFrame - the raw rs2_frame that can be used with the realsense2 library
    * @param depthData - the depth data as unsigned ints. This is a 1D array that represents a 2D depth image.

    * 
    * When you try to get data from the ShortBuffer, be sure to call:
    * int val = Short.toUnsignedInt(depthData.get(index));
    * 
    * int index = y * l515.getDepthWidth() + x;
    */
   public void receivedDepthData(rs2_frame depthFrame, ShortBuffer depthData);
}
