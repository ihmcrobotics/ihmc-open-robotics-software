package us.ihmc.vulkan;

import org.lwjgl.system.MemoryStack;
import org.lwjgl.vulkan.VK10;
import org.lwjgl.vulkan.VkVertexInputAttributeDescription;
import org.lwjgl.vulkan.VkVertexInputBindingDescription;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.color.MutableColor;

public class VulkanVertex
{
   public static final int SIZEOF = 2 * Float.BYTES + 3 * Float.BYTES;
   private static final int POSITION_OFFSET = 0;
   private static final int COLOR_OFFSET = 2 * Float.BYTES;

   private final Vector2D position;
   private final MutableColor color;

   public VulkanVertex(Vector2D position, MutableColor color)
   {
      this.position = position;
      this.color = color;
   }

   public Vector2D getPosition()
   {
      return position;
   }

   public MutableColor getColor()
   {
      return color;
   }

   public static VkVertexInputBindingDescription.Buffer getBindingDescription(MemoryStack stack)
   {
      VkVertexInputBindingDescription.Buffer bindingDescription = VkVertexInputBindingDescription.calloc(1, stack);

      bindingDescription.binding(0);
      bindingDescription.stride(SIZEOF);
      bindingDescription.inputRate(VK10.VK_VERTEX_INPUT_RATE_VERTEX);

      return bindingDescription;
   }

   public static VkVertexInputAttributeDescription.Buffer getAttributeDescriptions(MemoryStack stack)
   {
      VkVertexInputAttributeDescription.Buffer attributeDescriptions = VkVertexInputAttributeDescription.calloc(2);

      // Position
      VkVertexInputAttributeDescription posDescription = attributeDescriptions.get(0);
      posDescription.binding(0);
      posDescription.location(0);
      posDescription.format(VK10.VK_FORMAT_R32G32_SFLOAT);
      posDescription.offset(POSITION_OFFSET);

      // Color
      VkVertexInputAttributeDescription colorDescription = attributeDescriptions.get(1);
      colorDescription.binding(0);
      colorDescription.location(1);
      colorDescription.format(VK10.VK_FORMAT_R32G32B32_SFLOAT);
      colorDescription.offset(COLOR_OFFSET);

      return attributeDescriptions.rewind();
   }
}
