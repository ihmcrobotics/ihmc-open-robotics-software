package us.ihmc.perception.cuda;

import gnu.trove.list.array.TFloatArrayList;
import org.bytedeco.hdf5.H5FD_file_image_callbacks_t.Image_malloc_long_int_Pointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.perception.BytedecoImage;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;

public class CudaHeightMapTools
{
   public static void TransformToPointer(float[] transform, PointerPointer transformPointer)
   {
      // TODO make it an array pointer

   }

   public static void BytedecoImageToPointer(BytedecoImage bdimage, Image_malloc_long_int_Pointer imagePointer)
   {
      //TODO MAKE A BYTEDECO IMAGE A PONTER
   }

   public static void CUFileAsString(String[] filePath)
   {
      try
      {
         // Read the file content into a byte array
         byte[] bytes = Files.readAllBytes(Paths.get(Arrays.toString(filePath)));

         // Convert the byte array to a string
         String content = new String(bytes);

         // Print the content
         System.out.println(content);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}

