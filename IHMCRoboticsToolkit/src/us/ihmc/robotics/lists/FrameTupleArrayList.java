package us.ihmc.robotics.lists;

import java.util.List;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple3D;
import us.ihmc.robotics.geometry.FrameVector3D;

public class FrameTupleArrayList<T extends FrameTuple3D<?, ?>> extends RecyclingArrayList<T>
{
   private FrameTupleArrayList(Class<T> clazz)
   {
      super(clazz);
   }

   private FrameTupleArrayList(int initialSize, Class<T> clazz)
   {
      super(initialSize, clazz);
   }

   public void setOrCreate(int i, FrameTuple3D<?, ?> frameTuple)
   {
      if (i >= size)
      {
         size = i + 1;
         ensureCapacity(size);
      }
      unsafeGet(i).setIncludingFrame(frameTuple);
   }

   public void set(int i, FrameTuple3D<?, ?> frameTuple)
   {
      get(i).setIncludingFrame(frameTuple);
   }

   private void unsafeSet(int i, FrameTuple3D<?, ?> frameTuple)
   {
      unsafeGet(i).setIncludingFrame(frameTuple);
   }

   public void copyFromListAndTrimSize(FrameTupleArrayList<?> otherList)
   {
      ensureCapacity(otherList.size());
      size = otherList.size;

      for (int i = 0; i < size; i++)
      {
         unsafeSet(i, otherList.unsafeGet(i));
      }
   }

   public void copyFromListAndTrimSize(List<? extends FrameTuple3D<?, ?>> otherList)
   {
      ensureCapacity(otherList.size());
      size = otherList.size();

      for (int i = 0; i < size; i++)
      {
         unsafeSet(i, otherList.get(i));
      }
   }

   public static FrameTupleArrayList<FramePoint> createFramePointArrayList()
   {
      return new FrameTupleArrayList<>(FramePoint.class);
   }

   public static FrameTupleArrayList<FrameVector3D> createFrameVectorArrayList()
   {
      return new FrameTupleArrayList<>(FrameVector3D.class);
   }

   public static FrameTupleArrayList<FramePoint> createFramePointArrayList(int initialSize)
   {
      return new FrameTupleArrayList<>(initialSize, FramePoint.class);
   }

   public static FrameTupleArrayList<FrameVector3D> createFrameVectorArrayList(int initialSize)
   {
      return new FrameTupleArrayList<>(initialSize, FrameVector3D.class);
   }
}
