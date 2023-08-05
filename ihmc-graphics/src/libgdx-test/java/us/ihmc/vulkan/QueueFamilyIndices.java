package us.ihmc.vulkan;

public class QueueFamilyIndices
{
   // We use Integer to use null as the empty value
   private Integer graphicsFamily;

   public void setGraphicsFamily(Integer graphicsFamily)
   {
      this.graphicsFamily = graphicsFamily;
   }

   public boolean isComplete()
   {
      return graphicsFamily != null;
   }
}
