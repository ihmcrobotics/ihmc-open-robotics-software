package us.ihmc.avatar.logging;

public class PlanarRegionsBuffer<T>
{
   private T list;
   private long time;
   private int index;

   PlanarRegionsBuffer(int index, long time, T list)
   {
      this.list = list;
      this.index = index;
      this.time = time;
   }

   public T getList()
   {
      return list;
   }

   public long getTime()
   {
      return time;
   }

   public int getIndex()
   {
      return index;
   }
}