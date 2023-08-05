package us.ihmc.vulkan;

import org.lwjgl.system.Configuration;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.vulkan.*;

import java.nio.LongBuffer;
import java.util.HashSet;
import java.util.Set;

public class VulkanTools
{
   public static final boolean ENABLE_VALIDATION_LAYERS = Configuration.DEBUG.get(true);

   public static final Set<String> VALIDATION_LAYERS;

   static
   {
      if (ENABLE_VALIDATION_LAYERS)
      {
         VALIDATION_LAYERS = new HashSet<>();
         VALIDATION_LAYERS.add("VK_LAYER_KHRONOS_validation");
      }
      else
      {
         // We are not going to use it, so we don't create it
         VALIDATION_LAYERS = null;
      }
   }

   public static int debugCallback(int messageSeverity, int messageType, long pCallbackData, long pUserData)
   {
      VkDebugUtilsMessengerCallbackDataEXT callbackData = VkDebugUtilsMessengerCallbackDataEXT.create(pCallbackData);
      System.err.println("Validation layer: " + callbackData.pMessageString());
      return VK10.VK_FALSE;
   }

   public static int createDebugUtilsMessengerEXT(VkInstance instance,
                                                  VkDebugUtilsMessengerCreateInfoEXT createInfo,
                                                  VkAllocationCallbacks allocationCallbacks,
                                                  LongBuffer pDebugMessenger)
   {
      if (VK10.vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT") != MemoryUtil.NULL)
         return EXTDebugUtils.vkCreateDebugUtilsMessengerEXT(instance, createInfo, allocationCallbacks, pDebugMessenger);

      return VK10.VK_ERROR_EXTENSION_NOT_PRESENT;
   }

   public static void destroyDebugUtilsMessengerEXT(VkInstance instance, long debugMessenger, VkAllocationCallbacks allocationCallbacks)
   {
      if (VK10.vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT") != MemoryUtil.NULL)
         EXTDebugUtils.vkDestroyDebugUtilsMessengerEXT(instance, debugMessenger, allocationCallbacks);
   }
}
