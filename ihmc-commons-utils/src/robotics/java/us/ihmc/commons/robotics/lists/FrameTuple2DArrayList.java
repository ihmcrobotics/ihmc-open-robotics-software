package us.ihmc.commons.robotics.lists;

import java.util.List;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class FrameTuple2DArrayList<T extends FrameTuple2DBasics> extends RecyclingArrayList<T>
{
   private FrameTuple2DArrayList(Class<T> clazz)
   {
      super(clazz);
   }

   private FrameTuple2DArrayList(int initialSize, Class<T> clazz)
   {
      super(initialSize, clazz);
   }

   public void setOrCreate(int i, FrameTuple2DReadOnly frameTuple2d)
   {
      getAndGrowIfNeeded(i).setIncludingFrame(frameTuple2d);
   }

   public void set(int i, FrameTuple2DReadOnly frameTuple2d)
   {
      get(i).setIncludingFrame(frameTuple2d);
   }

   private void unsafeSet(int i, FrameTuple2DReadOnly frameTuple2d)
   {
      unsafeGet(i).setIncludingFrame(frameTuple2d);
   }

   private void unsafeSet(int i, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2d)
   {
      unsafeGet(i).setIncludingFrame(referenceFrame, tuple2d);
   }

   public void copyFromListAndTrimSize(FrameTuple2DArrayList<?> otherList)
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

   public void copyFromListAndTrimSize(List<? extends FrameTuple2DReadOnly> otherList)
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

   public void copyFromPoint2dListAndTrimSize(ReferenceFrame referenceFrame, List<? extends Tuple2DReadOnly> otherList)
   {
      for (int i = 0; i < otherList.size(); i++)
      {
         getAndGrowIfNeeded(i).setIncludingFrame(referenceFrame, otherList.get(i));
      }

      while(size() > otherList.size())
      {
         remove(size() - 1);
      }
   }

   public static FrameTuple2DArrayList<FramePoint2D> createFramePoint2dArrayList()
   {
      return new FrameTuple2DArrayList<>(FramePoint2D.class);
   }

   public static FrameTuple2DArrayList<FrameVector2D> createFrameVector2dArrayList()
   {
      return new FrameTuple2DArrayList<>(FrameVector2D.class);
   }

   public static FrameTuple2DArrayList<FramePoint2D> createFramePoint2dArrayList(int initialSize)
   {
      return new FrameTuple2DArrayList<>(initialSize, FramePoint2D.class);
   }

   public static FrameTuple2DArrayList<FrameVector2D> createFrameVector2dArrayList(int initialSize)
   {
      return new FrameTuple2DArrayList<>(initialSize, FrameVector2D.class);
   }
}
