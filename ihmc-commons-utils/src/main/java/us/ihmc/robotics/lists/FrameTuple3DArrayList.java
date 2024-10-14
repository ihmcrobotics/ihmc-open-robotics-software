package us.ihmc.robotics.lists;

import java.util.List;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

public class FrameTuple3DArrayList<T extends FrameTuple3DBasics> extends RecyclingArrayList<T>
{
   private FrameTuple3DArrayList(Class<T> clazz)
   {
      super(clazz);
   }

   private FrameTuple3DArrayList(int initialSize, Class<T> clazz)
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

   public void set(FrameTuple3DArrayList<?> otherList)
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
      for (int i = 0; i < otherList.size(); i++)
      {
         getAndGrowIfNeeded(i).setIncludingFrame(otherList.get(i));
      }

      while(size() > otherList.size())
      {
         remove(size() - 1);
      }
   }

   public static FrameTuple3DArrayList<FramePoint3D> createFramePointArrayList()
   {
      return new FrameTuple3DArrayList<>(FramePoint3D.class);
   }

   public static FrameTuple3DArrayList<FrameVector3D> createFrameVectorArrayList()
   {
      return new FrameTuple3DArrayList<>(FrameVector3D.class);
   }

   public static FrameTuple3DArrayList<FramePoint3D> createFramePointArrayList(int initialSize)
   {
      return new FrameTuple3DArrayList<>(initialSize, FramePoint3D.class);
   }

   public static FrameTuple3DArrayList<FrameVector3D> createFrameVectorArrayList(int initialSize)
   {
      return new FrameTuple3DArrayList<>(initialSize, FrameVector3D.class);
   }
}
