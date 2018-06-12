package us.ihmc.robotics.lists;

import java.util.List;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

public class FrameTupleArrayList<T extends FrameTuple3DBasics> extends RecyclingArrayList<T>
{
   private FrameTupleArrayList(Class<T> clazz)
   {
      super(clazz);
   }

   private FrameTupleArrayList(int initialSize, Class<T> clazz)
   {
      super(initialSize, clazz);
   }

   public void setOrCreate(int i, FrameTuple3DReadOnly frameTuple)
   {
      getAndGrowIfNeeded(i).setIncludingFrame(frameTuple);
   }

   public void set(int i, FrameTuple3DReadOnly frameTuple)
   {
      get(i).setIncludingFrame(frameTuple);
   }

   private void unsafeSet(int i, FrameTuple3DReadOnly frameTuple)
   {
      unsafeGet(i).setIncludingFrame(frameTuple);
   }

   public void copyFromListAndTrimSize(FrameTupleArrayList<?> otherList)
   {
      for (int i = 0; i < otherList.size(); i++)
      {
         getAndGrowIfNeeded(i).setIncludingFrame(otherList.get(i));
      }

      while(size() > otherList.size())
      {
         remove(size() - 1);
      }
   }

   public void copyFromListAndTrimSize(List<? extends FrameTuple3DReadOnly> otherList)
   {
      copyFromListAndTrimSize(otherList);
   }

   public static FrameTupleArrayList<FramePoint3D> createFramePointArrayList()
   {
      return new FrameTupleArrayList<>(FramePoint3D.class);
   }

   public static FrameTupleArrayList<FrameVector3D> createFrameVectorArrayList()
   {
      return new FrameTupleArrayList<>(FrameVector3D.class);
   }

   public static FrameTupleArrayList<FramePoint3D> createFramePointArrayList(int initialSize)
   {
      return new FrameTupleArrayList<>(initialSize, FramePoint3D.class);
   }

   public static FrameTupleArrayList<FrameVector3D> createFrameVectorArrayList(int initialSize)
   {
      return new FrameTupleArrayList<>(initialSize, FrameVector3D.class);
   }
}
