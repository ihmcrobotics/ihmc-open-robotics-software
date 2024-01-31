package us.ihmc.perception.opencl;

import org.bytedeco.javacpp.CharPointer;
import org.bytedeco.javacpp.SizeTPointer;

public interface OpenCLStringProvider
{
   void read(int stringSizeByteLimit, CharPointer stringPointer, SizeTPointer resultingStringLengthPointer);
}
