package us.ihmc.vulkan;

import java.nio.LongBuffer;

import static org.lwjgl.system.MemoryStack.stackGet;

/**
 * Wraps the needed sync objects for an in flight frame
 * <p>
 * This frame's sync objects must be deleted manually
 */
public class HelloVulkanFrame
{
   private final long imageAvailableSemaphore;
   private final long renderFinishedSemaphore;
   private final long fence;

   public HelloVulkanFrame(long imageAvailableSemaphore, long renderFinishedSemaphore, long fence)
   {
      this.imageAvailableSemaphore = imageAvailableSemaphore;
      this.renderFinishedSemaphore = renderFinishedSemaphore;
      this.fence = fence;
   }

   public long imageAvailableSemaphore()
   {
      return imageAvailableSemaphore;
   }

   public LongBuffer pImageAvailableSemaphore()
   {
      return stackGet().longs(imageAvailableSemaphore);
   }

   public long renderFinishedSemaphore()
   {
      return renderFinishedSemaphore;
   }

   public LongBuffer pRenderFinishedSemaphore()
   {
      return stackGet().longs(renderFinishedSemaphore);
   }

   public long fence()
   {
      return fence;
   }

   public LongBuffer pFence()
   {
      return stackGet().longs(fence);
   }
}