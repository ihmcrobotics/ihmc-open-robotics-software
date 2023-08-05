package us.ihmc.vulkan;

import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWVulkan;
import org.lwjgl.system.Configuration;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.vulkan.*;

import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Taken from https://github.com/Naitsirc98/Vulkan-Tutorial-Java
 */
public class HelloVulkan
{
   private static final int WIDTH = 800;
   private static final int HEIGHT = 600;

   private static final boolean ENABLE_VALIDATION_LAYERS = Configuration.DEBUG.get(true);

   private static final Set<String> VALIDATION_LAYERS;

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

   private static int debugCallback(int messageSeverity, int messageType, long pCallbackData, long pUserData)
   {
      VkDebugUtilsMessengerCallbackDataEXT callbackData = VkDebugUtilsMessengerCallbackDataEXT.create(pCallbackData);
      System.err.println("Validation layer: " + callbackData.pMessageString());
      return VK10.VK_FALSE;
   }

   private static int createDebugUtilsMessengerEXT(VkInstance instance,
                                                   VkDebugUtilsMessengerCreateInfoEXT createInfo,
                                                   VkAllocationCallbacks allocationCallbacks,
                                                   LongBuffer pDebugMessenger)
   {
      if (VK10.vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT") != MemoryUtil.NULL)
         return EXTDebugUtils.vkCreateDebugUtilsMessengerEXT(instance, createInfo, allocationCallbacks, pDebugMessenger);

      return VK10.VK_ERROR_EXTENSION_NOT_PRESENT;
   }

   private static void destroyDebugUtilsMessengerEXT(VkInstance instance, long debugMessenger, VkAllocationCallbacks allocationCallbacks)
   {
      if (VK10.vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT") != MemoryUtil.NULL)
         EXTDebugUtils.vkDestroyDebugUtilsMessengerEXT(instance, debugMessenger, allocationCallbacks);
   }

   private long window;
   private VkInstance vulkanInstance;
   private long debugMessenger;

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
      if (ENABLE_VALIDATION_LAYERS && !checkValidationLayerSupport())
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

         if (ENABLE_VALIDATION_LAYERS)
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

      setupDebugMessenger();
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
      if (ENABLE_VALIDATION_LAYERS)
         destroyDebugUtilsMessengerEXT(vulkanInstance, debugMessenger, null);

      VK10.vkDestroyInstance(vulkanInstance, null);

      GLFW.glfwDestroyWindow(window);
      GLFW.glfwTerminate();
   }

   private void setupDebugMessenger()
   {
      if (!ENABLE_VALIDATION_LAYERS)
      {
         return;
      }

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkDebugUtilsMessengerCreateInfoEXT createInfo = VkDebugUtilsMessengerCreateInfoEXT.calloc(stack);
         populateDebugMessengerCreateInfo(createInfo);
         LongBuffer pDebugMessenger = stack.longs(VK10.VK_NULL_HANDLE);

         if (createDebugUtilsMessengerEXT(vulkanInstance, createInfo, null, pDebugMessenger) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to set up debug messenger");
         }

         debugMessenger = pDebugMessenger.get(0);
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
      debugCreateInfo.pfnUserCallback(HelloVulkan::debugCallback);
   }

   private PointerBuffer validationLayersAsPointerBuffer(MemoryStack stack)
   {
      PointerBuffer buffer = stack.mallocPointer(VALIDATION_LAYERS.size());
      VALIDATION_LAYERS.stream().map(stack::UTF8).forEach(buffer::put);
      return buffer.rewind();
   }

   private PointerBuffer getRequiredExtensions(MemoryStack stack)
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

   private boolean checkValidationLayerSupport()
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

   public static void main(String[] args)
   {
      new HelloVulkan().run();
   }
}
