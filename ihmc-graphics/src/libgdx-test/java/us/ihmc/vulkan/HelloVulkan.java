package us.ihmc.vulkan;

import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWVulkan;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.vulkan.*;
import us.ihmc.log.LogTools;

import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Taken from https://github.com/Naitsirc98/Vulkan-Tutorial-Java
 */
public class HelloVulkan
{
   private static final int WIDTH = 800;
   private static final int HEIGHT = 600;

   private long window;
   private VkInstance vulkanInstance;
   private long debugMessenger;
   private VkPhysicalDevice physicalDevice;

   public void run()
   {
      initWindow();
      initVulkan();
      mainLoop();
      cleanup();
   }

   private void initWindow()
   {
      if (!GLFW.glfwInit())
      {
         throw new RuntimeException("Cannot initialize GLFW");
      }

      GLFW.glfwWindowHint(GLFW.GLFW_CLIENT_API, GLFW.GLFW_NO_API);
      GLFW.glfwWindowHint(GLFW.GLFW_RESIZABLE, GLFW.GLFW_FALSE);

      String title = HelloVulkan.class.getSimpleName();
      window = GLFW.glfwCreateWindow(WIDTH, HEIGHT, title, MemoryUtil.NULL, MemoryUtil.NULL);

      if (window == MemoryUtil.NULL)
      {
         throw new RuntimeException("Cannot create window");
      }
   }

   private void initVulkan()
   {
      createInstance();
      setupDebugMessenger();
      pickPhysicalDevice();
   }

   private void mainLoop()
   {
      while (!GLFW.glfwWindowShouldClose(window))
      {
         GLFW.glfwPollEvents();
      }
   }

   private void cleanup()
   {
      if (VulkanTools.ENABLE_VALIDATION_LAYERS)
         VulkanTools.destroyDebugUtilsMessengerEXT(vulkanInstance, debugMessenger, null);

      VK10.vkDestroyInstance(vulkanInstance, null);

      GLFW.glfwDestroyWindow(window);
      GLFW.glfwTerminate();
   }

   private void createInstance()
   {
      if (VulkanTools.ENABLE_VALIDATION_LAYERS && !checkValidationLayerSupport())
         throw new RuntimeException("Validation requested but not supported. Make sure to install the vulkan validation layers on your system");

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         // Use calloc to initialize the structs with 0s. Otherwise, the program can crash due to random values
         VkApplicationInfo applicationInfo = VkApplicationInfo.calloc(stack);

         applicationInfo.sType(VK10.VK_STRUCTURE_TYPE_APPLICATION_INFO);
         applicationInfo.pApplicationName(stack.UTF8Safe("Hello Triangle"));
         applicationInfo.applicationVersion(VK10.VK_MAKE_VERSION(1, 0, 0));
         applicationInfo.pEngineName(stack.UTF8Safe("No Engine"));
         applicationInfo.engineVersion(VK10.VK_MAKE_VERSION(1, 0, 0));
         applicationInfo.apiVersion(VK10.VK_API_VERSION_1_0);

         VkInstanceCreateInfo createInfo = VkInstanceCreateInfo.calloc(stack);

         createInfo.sType(VK10.VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO);
         createInfo.pApplicationInfo(applicationInfo);
         // enabledExtensionCount is implicitly set when you call ppEnabledExtensionNames
         createInfo.ppEnabledExtensionNames(getRequiredExtensions(stack));

         if (VulkanTools.ENABLE_VALIDATION_LAYERS)
         {
            createInfo.ppEnabledLayerNames(validationLayersAsPointerBuffer(stack));

            VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo = VkDebugUtilsMessengerCreateInfoEXT.calloc(stack);
            populateDebugMessengerCreateInfo(debugCreateInfo);
            createInfo.pNext(debugCreateInfo.address());
         }

         // We need to retrieve the pointer of the created instance
         PointerBuffer instancePointerBuffer = stack.mallocPointer(1);

         if (VK10.vkCreateInstance(createInfo, null, instancePointerBuffer) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create instance");
         }

         vulkanInstance = new VkInstance(instancePointerBuffer.get(0), createInfo);
      }
   }

   private void setupDebugMessenger()
   {
      if (!VulkanTools.ENABLE_VALIDATION_LAYERS)
      {
         return;
      }

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkDebugUtilsMessengerCreateInfoEXT createInfo = VkDebugUtilsMessengerCreateInfoEXT.calloc(stack);
         populateDebugMessengerCreateInfo(createInfo);
         LongBuffer pDebugMessenger = stack.longs(VK10.VK_NULL_HANDLE);

         if (VulkanTools.createDebugUtilsMessengerEXT(vulkanInstance, createInfo, null, pDebugMessenger) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to set up debug messenger");
         }

         debugMessenger = pDebugMessenger.get(0);
      }
   }

   private void pickPhysicalDevice()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         IntBuffer deviceCount = stack.ints(0);
         VK10.vkEnumeratePhysicalDevices(vulkanInstance, deviceCount, null);
         if (deviceCount.get(0) == 0)
         {
            throw new RuntimeException("Failed to find GPUs with Vulkan support");
         }

         PointerBuffer ppPhysicalDevices = stack.mallocPointer(deviceCount.get(0));
         VK10.vkEnumeratePhysicalDevices(vulkanInstance, deviceCount, ppPhysicalDevices);
         for (int i = 0; i < ppPhysicalDevices.capacity(); i++)
         {
            VkPhysicalDevice device = new VkPhysicalDevice(ppPhysicalDevices.get(i), vulkanInstance);

            if (isDeviceSuitable(device))
            {
               VkPhysicalDeviceProperties deviceProperties = VkPhysicalDeviceProperties.create();
               VK10.vkGetPhysicalDeviceProperties(device, deviceProperties);

               LogTools.info("Using {}", deviceProperties.deviceNameString());

               physicalDevice = device;
               return;
            }
         }

         throw new RuntimeException("Failed to find a suitable GPU");
      }
   }

   private boolean isDeviceSuitable(VkPhysicalDevice device)
   {
      QueueFamilyIndices indices = findQueueFamilies(device);
      return indices.isComplete();
   }

   private QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device)
   {
      QueueFamilyIndices indices = new QueueFamilyIndices();

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         IntBuffer queueFamilyCount = stack.ints(0);
         VK10.vkGetPhysicalDeviceQueueFamilyProperties(device, queueFamilyCount, null);

         VkQueueFamilyProperties.Buffer queueFamilies = VkQueueFamilyProperties.malloc(queueFamilyCount.get(0), stack);
         VK10.vkGetPhysicalDeviceQueueFamilyProperties(device, queueFamilyCount, queueFamilies);

         IntStream.range(0, queueFamilies.capacity())
                  .filter(index -> (queueFamilies.get(index).queueFlags() & VK10.VK_QUEUE_GRAPHICS_BIT) != 0)
                  .findFirst()
                  .ifPresent(indices::setGraphicsFamily);
         return indices;
      }
   }

   private void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo)
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

   private PointerBuffer validationLayersAsPointerBuffer(MemoryStack stack)
   {
      PointerBuffer buffer = stack.mallocPointer(VulkanTools.VALIDATION_LAYERS.size());
      VulkanTools.VALIDATION_LAYERS.stream().map(stack::UTF8).forEach(buffer::put);
      return buffer.rewind();
   }

   private PointerBuffer getRequiredExtensions(MemoryStack stack)
   {
      PointerBuffer glfwExtensions = GLFWVulkan.glfwGetRequiredInstanceExtensions();
      if (VulkanTools.ENABLE_VALIDATION_LAYERS)
      {
         PointerBuffer extensions = stack.mallocPointer(glfwExtensions.capacity() + 1);
         extensions.put(glfwExtensions);
         extensions.put(stack.UTF8(EXTDebugUtils.VK_EXT_DEBUG_UTILS_EXTENSION_NAME));
         // Rewind the buffer before returning it to reset its position back to 0
         return extensions.rewind();
      }
      return glfwExtensions;
   }

   private boolean checkValidationLayerSupport()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         IntBuffer layerCount = stack.ints(0);
         VK10.vkEnumerateInstanceLayerProperties(layerCount, null);
         VkLayerProperties.Buffer availableLayers = VkLayerProperties.malloc(layerCount.get(0), stack);
         VK10.vkEnumerateInstanceLayerProperties(layerCount, availableLayers);
         Set<String> availableLayerNames = availableLayers.stream().map(VkLayerProperties::layerNameString).collect(Collectors.toSet());
         return availableLayerNames.containsAll(VulkanTools.VALIDATION_LAYERS);
      }
   }

   public static void main(String[] args)
   {
      new HelloVulkan().run();
   }
}
