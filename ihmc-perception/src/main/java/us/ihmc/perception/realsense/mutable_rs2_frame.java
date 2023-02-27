package us.ihmc.perception.realsense;

import org.bytedeco.librealsense2.rs2_frame;

public class mutable_rs2_frame extends rs2_frame
{
   public void address(long address)
   {
      this.address = address;
      this.position = 0;
      this.limit = 0;
      this.capacity = 0;
   }
}
