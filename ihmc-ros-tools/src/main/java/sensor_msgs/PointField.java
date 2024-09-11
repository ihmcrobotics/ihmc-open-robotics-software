//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package sensor_msgs;

import org.ros.internal.message.Message;

public interface PointField extends Message {
   String _TYPE = "sensor_msgs/PointField";
   String _DEFINITION = "# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field\n";
   byte INT8 = 1;
   byte UINT8 = 2;
   byte INT16 = 3;
   byte UINT16 = 4;
   byte INT32 = 5;
   byte UINT32 = 6;
   byte FLOAT32 = 7;
   byte FLOAT64 = 8;

   String getName();

   void setName(String var1);

   int getOffset();

   void setOffset(int var1);

   byte getDatatype();

   void setDatatype(byte var1);

   int getCount();

   void setCount(int var1);
}
