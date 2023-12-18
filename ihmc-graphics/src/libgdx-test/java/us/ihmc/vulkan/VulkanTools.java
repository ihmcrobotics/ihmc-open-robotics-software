package us.ihmc.vulkan;

import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFWVulkan;
import org.lwjgl.system.Configuration;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.system.Pointer;
import org.lwjgl.vulkan.*;

import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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

   public static final Set<String> DEVICE_EXTENSIONS = Stream.of(KHRSwapchain.VK_KHR_SWAPCHAIN_EXTENSION_NAME).collect(Collectors.toSet());

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

   public static void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo)
   {
      debugCreateInfo.sType(EXTDebugUtils.VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT);
      debugCreateInfo.messageSeverity(EXTDebugUtils.VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT
                                    | EXTDebugUtils.VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
                                    | EXTDebugUtils.VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT);
      debugCreateInfo.messageType(EXTDebugUtils.VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                                | EXTDebugUtils.VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                                | EXTDebugUtils.VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT);
      debugCreateInfo.pfnUserCallback(VulkanTools::debugCallback);
   }

   public static PointerBuffer validationLayersAsPointerBuffer(MemoryStack stack)
   {
      PointerBuffer buffer = stack.mallocPointer(VALIDATION_LAYERS.size());
      VALIDATION_LAYERS.stream().map(stack::UTF8).forEach(buffer::put);
      return buffer.rewind();
   }

   public static PointerBuffer asPointerBuffer(MemoryStack stack, Collection<String> collection)
   {
      PointerBuffer buffer = stack.mallocPointer(collection.size());
      collection.stream().map(stack::UTF8).forEach(buffer::put);
      return buffer.rewind();
   }

   public static PointerBuffer asPointerBuffer(MemoryStack stack, List<? extends Pointer> list)
   {
      PointerBuffer buffer = stack.mallocPointer(list.size());
      list.forEach(buffer::put);
      return buffer.rewind();
   }

   public static PointerBuffer getRequiredExtensions(MemoryStack stack)
   {
      PointerBuffer glfwExtensions = GLFWVulkan.glfwGetRequiredInstanceExtensions();
      if (ENABLE_VALIDATION_LAYERS)
      {
         PointerBuffer extensions = stack.mallocPointer(glfwExtensions.capacity() + 1);
         extensions.put(glfwExtensions);
         extensions.put(stack.UTF8(EXTDebugUtils.VK_EXT_DEBUG_UTILS_EXTENSION_NAME));
         // Rewind the buffer before returning it to reset its position back to 0
         return extensions.rewind();
      }
      return glfwExtensions;
   }

   public static boolean checkValidationLayerSupport()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         IntBuffer layerCount = stack.ints(0);
         VK10.vkEnumerateInstanceLayerProperties(layerCount, null);
         VkLayerProperties.Buffer availableLayers = VkLayerProperties.malloc(layerCount.get(0), stack);
         VK10.vkEnumerateInstanceLayerProperties(layerCount, availableLayers);
         Set<String> availableLayerNames = availableLayers.stream().map(VkLayerProperties::layerNameString).collect(Collectors.toSet());
         return availableLayerNames.containsAll(VALIDATION_LAYERS);
      }
   }

   public static boolean checkDeviceExtensionSupport(VkPhysicalDevice device)
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         IntBuffer extensionCount = stack.ints(0);
         VK10.vkEnumerateDeviceExtensionProperties(device, (String) null, extensionCount, null);
         VkExtensionProperties.Buffer availableExtensions = VkExtensionProperties.malloc(extensionCount.get(0), stack);
         VK10.vkEnumerateDeviceExtensionProperties(device, (String) null, extensionCount, availableExtensions);
         return availableExtensions.stream().map(VkExtensionProperties::extensionNameString).collect(Collectors.toSet()).containsAll(DEVICE_EXTENSIONS);
      }
   }

   public static SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device, MemoryStack stack, long surface)
   {
      SwapChainSupportDetails details = new SwapChainSupportDetails();
      details.setCapabilities(VkSurfaceCapabilitiesKHR.malloc(stack));
      KHRSurface.vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, details.getCapabilities());

      IntBuffer count = stack.ints(0);
      KHRSurface.vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, count, null);
      if (count.get(0) != 0)
      {
         details.setFormats(VkSurfaceFormatKHR.malloc(count.get(0), stack));
         KHRSurface.vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, count, details.getFormats());
      }

      KHRSurface.vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, count, null);
      if (count.get(0) != 0)
      {
         details.setPresentModes(stack.mallocInt(count.get(0)));
         KHRSurface.vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, count, details.getPresentModes());
      }

      return details;
   }
}
