package us.ihmc.perception.cuda;

import gnu.trove.list.array.TFloatArrayList;
import org.bytedeco.hdf5.H5FD_file_image_callbacks_t.Image_malloc_long_int_Pointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.perception.BytedecoImage;

public class CudaHeightMapTools
{
   public static void TransformToPointer(float[] transform, PointerPointer transformPointer){
      // TODO make it an array pointer

   }
 public static void BytedecoImageToPointer(BytedecoImage bdimage , Image_malloc_long_int_Pointer imagePointer ){
      //TODO MAKE A BYTEDECO IMAGE A PONTER
 }
}
