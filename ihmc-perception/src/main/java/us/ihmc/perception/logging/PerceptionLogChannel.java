package us.ihmc.perception.logging;

public class PerceptionLogChannel
{
   private String name;
   private int count;
   private int index;
   boolean enabled = false;

   public PerceptionLogChannel(String name, int count, int index)
   {
      this.name = name;
      this.count = count;
      this.index = index;
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public String getName()
   {
      return name;
   }

   public int getCount()
   {
      return count;
   }

   public int getIndex()
   {
      return index;
   }

   public void setIndex(int index)
   {
      this.index = index;
   }

   public void incrementIndex()
   {
      index++;
   }

   public void incrementCount()
   {
      count++;
   }

   public void resetIndex()
   {
      index = 0;
   }

   public void resetCount()
   {
      count = 0;
   }


   public void setCount(int count)
   {
      this.count = count;
   }
}
