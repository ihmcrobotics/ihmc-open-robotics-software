package us.ihmc.perception.cuda.dataTypes;

import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;

public class CUDAIntegerArray extends PointerPointer<IntPointer>
{
   private final IntPointer javaSideValueContainer;

   public CUDAIntegerArray(long arraySize)
   {
       super(arraySize);

       javaSideValueContainer = new IntPointer(arraySize);
       this.put(javaSideValueContainer);
   }

   public int getFirst()
   {
      return javaSideValueContainer.get();
   }

   public int get(int element)
   {
      return javaSideValueContainer.get(element);
   }

   @Override
   public void close()
   {
      javaSideValueContainer.close();
      super.close();
   }
}
