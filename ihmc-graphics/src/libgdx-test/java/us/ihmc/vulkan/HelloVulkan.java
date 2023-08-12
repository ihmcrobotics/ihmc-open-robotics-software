package us.ihmc.vulkan;

import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWVulkan;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.vulkan.*;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * Taken from https://github.com/Naitsirc98/Vulkan-Tutorial-Java
 */
public class HelloVulkan
{
   private static final int UINT32_MAX = 0xFFFFFFFF;
   private static final int WIDTH = 800;
   private static final int HEIGHT = 600;

   private long window;
   private VkInstance vulkanInstance;
   private long debugMessenger;
   private long surface;
   private VkPhysicalDevice physicalDevice;
   private VkDevice device;
   private VkQueue graphicsQueue;
   private VkQueue presentQueue;
   private long swapChain;
   private List<Long> swapChainImages;
   private List<Long> swapChainImageViews;
   private int swapChainImageFormat;
   private VkExtent2D swapChainExtent;
   private long pipelineLayout;

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
      createSurface();
      pickPhysicalDevice();
      createLogicalDevice();
      createSwapChain();
      createImageViews();
      createGraphicsPipeline();
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
      VK10.vkDestroyPipelineLayout(device, pipelineLayout, null);

      swapChainImageViews.forEach(imageView -> VK10.vkDestroyImageView(device, imageView, null));
      KHRSwapchain.vkDestroySwapchainKHR(device, swapChain, null);

      VK10.vkDestroyDevice(device, null);

      if (VulkanTools.ENABLE_VALIDATION_LAYERS)
         VulkanTools.destroyDebugUtilsMessengerEXT(vulkanInstance, debugMessenger, null);

      KHRSurface.vkDestroySurfaceKHR(vulkanInstance, surface, null);

      VK10.vkDestroyInstance(vulkanInstance, null);

      GLFW.glfwDestroyWindow(window);
      GLFW.glfwTerminate();
   }

   private void createInstance()
   {
      if (VulkanTools.ENABLE_VALIDATION_LAYERS && !VulkanTools.checkValidationLayerSupport())
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
         createInfo.ppEnabledExtensionNames(VulkanTools.getRequiredExtensions(stack));

         if (VulkanTools.ENABLE_VALIDATION_LAYERS)
         {
            createInfo.ppEnabledLayerNames(VulkanTools.asPointerBuffer(stack, VulkanTools.VALIDATION_LAYERS));

            VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo = VkDebugUtilsMessengerCreateInfoEXT.calloc(stack);
            VulkanTools.populateDebugMessengerCreateInfo(debugCreateInfo);
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
         VulkanTools.populateDebugMessengerCreateInfo(createInfo);
         LongBuffer pDebugMessenger = stack.longs(VK10.VK_NULL_HANDLE);

         if (VulkanTools.createDebugUtilsMessengerEXT(vulkanInstance, createInfo, null, pDebugMessenger) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to set up debug messenger");
         }

         debugMessenger = pDebugMessenger.get(0);
      }
   }

   private void createSurface()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         LongBuffer pSurface = stack.longs(VK10.VK_NULL_HANDLE);
         if (GLFWVulkan.glfwCreateWindowSurface(vulkanInstance, window, null, pSurface) != VK10.VK_SUCCESS)
            throw new RuntimeException("Failed to create window surface");
         surface = pSurface.get(0);
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
               VkPhysicalDeviceProperties deviceProperties = VkPhysicalDeviceProperties.calloc(stack);
               VK10.vkGetPhysicalDeviceProperties(device, deviceProperties);
               LogTools.info("Using {}", deviceProperties.deviceNameString());


               physicalDevice = device;
               return;
            }
         }

         throw new RuntimeException("Failed to find a suitable GPU");
      }
   }

   private void createLogicalDevice()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
         int[] uniqueQueueFamilies = indices.unique();
         VkDeviceQueueCreateInfo.Buffer queueCreateInfos = VkDeviceQueueCreateInfo.calloc(uniqueQueueFamilies.length, stack);

         for (int i = 0; i < uniqueQueueFamilies.length; i++)
         {
            VkDeviceQueueCreateInfo queueCreateInfo = queueCreateInfos.get(i);
            queueCreateInfo.sType(VK10.VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO);
            queueCreateInfo.queueFamilyIndex(uniqueQueueFamilies[i]);
            queueCreateInfo.pQueuePriorities(stack.floats(1.0f));
         }

         VkPhysicalDeviceFeatures deviceFeatures = VkPhysicalDeviceFeatures.calloc(stack);

         VkDeviceCreateInfo createInfo = VkDeviceCreateInfo.calloc(stack);
         createInfo.sType(VK10.VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO);
         createInfo.pQueueCreateInfos(queueCreateInfos);
         // queueCreateInfoCount is automatically set
         createInfo.pEnabledFeatures(deviceFeatures);
         createInfo.ppEnabledExtensionNames(VulkanTools.asPointerBuffer(stack, VulkanTools.DEVICE_EXTENSIONS));
         if (VulkanTools.ENABLE_VALIDATION_LAYERS)
            createInfo.ppEnabledLayerNames(VulkanTools.validationLayersAsPointerBuffer(stack));

         PointerBuffer pDevice = stack.pointers(VK10.VK_NULL_HANDLE);
         if (VK10.vkCreateDevice(physicalDevice, createInfo, null, pDevice) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create logical device");
         }

         device = new VkDevice(pDevice.get(0), physicalDevice, createInfo);
         PointerBuffer pQueue = stack.pointers(VK10.VK_NULL_HANDLE);
         VK10.vkGetDeviceQueue(device, indices.getGraphicsFamily(), 0, pQueue);
         graphicsQueue = new VkQueue(pQueue.get(0), device);
         VK10.vkGetDeviceQueue(device, indices.getPresentFamily(), 0, pQueue);
         presentQueue = new VkQueue(pQueue.get(0), device);
      }
   }

   private void createSwapChain()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         SwapChainSupportDetails swapChainSupport = VulkanTools.querySwapChainSupport(physicalDevice, stack, surface);
         VkSurfaceFormatKHR surfaceFormat = chooseSwapSurfaceFormat(swapChainSupport.getFormats());
         int presentMode = chooseSwapPresentMode(swapChainSupport.getPresentModes());
         VkExtent2D extent = chooseSwapExtent(stack, swapChainSupport.getCapabilities());

         IntBuffer imageCount = stack.ints(swapChainSupport.getCapabilities().minImageCount() + 1);
         if (swapChainSupport.getCapabilities().maxImageCount() > 0 && imageCount.get(0) > swapChainSupport.getCapabilities().maxImageCount())
         {
            imageCount.put(0, swapChainSupport.getCapabilities().maxImageCount());
         }

         VkSwapchainCreateInfoKHR createInfo = VkSwapchainCreateInfoKHR.calloc(stack);
         createInfo.sType(KHRSwapchain.VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR);
         createInfo.surface(surface);
         // Image settings
         createInfo.minImageCount(imageCount.get(0));
         createInfo.imageFormat(surfaceFormat.format());
         createInfo.imageColorSpace(surfaceFormat.colorSpace());
         createInfo.imageExtent(extent);
         createInfo.imageArrayLayers(1);
         createInfo.imageUsage(VK10.VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
         QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
         if (!indices.getGraphicsFamily().equals(indices.getPresentFamily()))
         {
            createInfo.imageSharingMode(VK10.VK_SHARING_MODE_CONCURRENT);
            createInfo.pQueueFamilyIndices(stack.ints(indices.getGraphicsFamily(), indices.getPresentFamily()));
         }
         else
         {
            createInfo.imageSharingMode(VK10.VK_SHARING_MODE_EXCLUSIVE);
         }
         createInfo.preTransform(swapChainSupport.getCapabilities().currentTransform());
         createInfo.compositeAlpha(KHRSurface.VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR);
         createInfo.presentMode(presentMode);
         createInfo.clipped(true);
         createInfo.oldSwapchain(VK10.VK_NULL_HANDLE);

         LongBuffer pSwapChain = stack.longs(VK10.VK_NULL_HANDLE);
         if (KHRSwapchain.vkCreateSwapchainKHR(device, createInfo, null, pSwapChain) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create swap chain");
         }

         swapChain = pSwapChain.get(0);
         KHRSwapchain.vkGetSwapchainImagesKHR(device, swapChain, imageCount, null);
         LongBuffer pSwapchainImages = stack.mallocLong(imageCount.get(0));
         KHRSwapchain.vkGetSwapchainImagesKHR(device, swapChain, imageCount, pSwapchainImages);

         swapChainImages = new ArrayList<>(imageCount.get(0));
         for (int i = 0; i < pSwapchainImages.capacity(); i++)
         {
            swapChainImages.add(pSwapchainImages.get(i));
         }

         swapChainImageFormat = surfaceFormat.format();
         swapChainExtent = VkExtent2D.create().set(extent);
      }
   }

   private void createImageViews()
   {
      swapChainImageViews = new ArrayList<>(swapChainImages.size());
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         LongBuffer pImageView = stack.mallocLong(1);

         for (long swapChainImage : swapChainImages)
         {
            VkImageViewCreateInfo createInfo = VkImageViewCreateInfo.calloc(stack);
            createInfo.sType(VK10.VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
            createInfo.image(swapChainImage);
            createInfo.viewType(VK10.VK_IMAGE_VIEW_TYPE_2D);
            createInfo.format(swapChainImageFormat);
            createInfo.components().r(VK10.VK_COMPONENT_SWIZZLE_IDENTITY);
            createInfo.components().g(VK10.VK_COMPONENT_SWIZZLE_IDENTITY);
            createInfo.components().b(VK10.VK_COMPONENT_SWIZZLE_IDENTITY);
            createInfo.components().a(VK10.VK_COMPONENT_SWIZZLE_IDENTITY);
            createInfo.subresourceRange().aspectMask(VK10.VK_IMAGE_ASPECT_COLOR_BIT);
            createInfo.subresourceRange().baseMipLevel(0);
            createInfo.subresourceRange().levelCount(1);
            createInfo.subresourceRange().baseArrayLayer(0);
            createInfo.subresourceRange().layerCount(1);

            if (VK10.vkCreateImageView(device, createInfo, null, pImageView) != VK10.VK_SUCCESS)
               throw new RuntimeException("Failed to create image views");

            swapChainImageViews.add(pImageView.get(0));
         }
      }
   }

   private void createGraphicsPipeline()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         // Let's compile the GLSL shaders into SPIR-V at runtime using the shaderc library
         // Check ShaderSPIRVUtils class to see how it can be done
         ShaderSPIRVUtils.SPIRV vertShaderSPIRV
               = ShaderSPIRVUtils.compileShaderFile("shaders/09_shader_base.vert", ShaderSPIRVUtils.ShaderKind.VERTEX_SHADER);
         ShaderSPIRVUtils.SPIRV fragShaderSPIRV
               = ShaderSPIRVUtils.compileShaderFile("shaders/09_shader_base.frag", ShaderSPIRVUtils.ShaderKind.FRAGMENT_SHADER);
         long vertShaderModule = createShaderModule(vertShaderSPIRV.bytecode());
         long fragShaderModule = createShaderModule(fragShaderSPIRV.bytecode());

         ByteBuffer entryPoint = stack.UTF8("main");
         VkPipelineShaderStageCreateInfo.Buffer shaderStages = VkPipelineShaderStageCreateInfo.calloc(2, stack);

         VkPipelineShaderStageCreateInfo vertShaderStageInfo = shaderStages.get(0);
         vertShaderStageInfo.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO);
         vertShaderStageInfo.stage(VK10.VK_SHADER_STAGE_VERTEX_BIT);
         vertShaderStageInfo.module(vertShaderModule);
         vertShaderStageInfo.pName(entryPoint);

         VkPipelineShaderStageCreateInfo fragShaderStageInfo = shaderStages.get(1);
         fragShaderStageInfo.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO);
         fragShaderStageInfo.stage(VK10.VK_SHADER_STAGE_FRAGMENT_BIT);
         fragShaderStageInfo.module(fragShaderModule);
         fragShaderStageInfo.pName(entryPoint);

         // ===> VERTEX STAGE <===

         VkPipelineVertexInputStateCreateInfo vertexInputInfo = VkPipelineVertexInputStateCreateInfo.calloc(stack);
         vertexInputInfo.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO);

         // ===> ASSEMBLY STAGE <===

         VkPipelineInputAssemblyStateCreateInfo inputAssembly = VkPipelineInputAssemblyStateCreateInfo.calloc(stack);
         inputAssembly.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO);
         inputAssembly.topology(VK10.VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
         inputAssembly.primitiveRestartEnable(false);

         // ===> VIEWPORT & SCISSOR

         VkViewport.Buffer viewport = VkViewport.calloc(1, stack);
         viewport.x(0.0f);
         viewport.y(0.0f);
         viewport.width(swapChainExtent.width());
         viewport.height(swapChainExtent.height());
         viewport.minDepth(0.0f);
         viewport.maxDepth(1.0f);

         VkRect2D.Buffer scissor = VkRect2D.calloc(1, stack);
         scissor.offset(VkOffset2D.calloc(stack).set(0, 0));
         scissor.extent(swapChainExtent);

         VkPipelineViewportStateCreateInfo viewportState = VkPipelineViewportStateCreateInfo.calloc(stack);
         viewportState.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO);
         viewportState.pViewports(viewport);
         viewportState.pScissors(scissor);

         // ===> RASTERIZATION STAGE <===

         VkPipelineRasterizationStateCreateInfo rasterizer = VkPipelineRasterizationStateCreateInfo.calloc(stack);
         rasterizer.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO);
         rasterizer.depthClampEnable(false);
         rasterizer.rasterizerDiscardEnable(false);
         rasterizer.polygonMode(VK10.VK_POLYGON_MODE_FILL);
         rasterizer.lineWidth(1.0f);
         rasterizer.cullMode(VK10.VK_CULL_MODE_BACK_BIT);
         rasterizer.frontFace(VK10.VK_FRONT_FACE_CLOCKWISE);
         rasterizer.depthBiasEnable(false);

         // ===> MULTISAMPLING <===

         VkPipelineMultisampleStateCreateInfo multisampling = VkPipelineMultisampleStateCreateInfo.calloc(stack);
         multisampling.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO);
         multisampling.sampleShadingEnable(false);
         multisampling.rasterizationSamples(VK10.VK_SAMPLE_COUNT_1_BIT);

         // ===> COLOR BLENDING <===

         VkPipelineColorBlendAttachmentState.Buffer colorBlendAttachment = VkPipelineColorBlendAttachmentState.calloc(1, stack);
         colorBlendAttachment.colorWriteMask(VK10.VK_COLOR_COMPONENT_R_BIT
                                           | VK10.VK_COLOR_COMPONENT_G_BIT
                                           | VK10.VK_COLOR_COMPONENT_B_BIT
                                           | VK10.VK_COLOR_COMPONENT_A_BIT);
         colorBlendAttachment.blendEnable(false);

         VkPipelineColorBlendStateCreateInfo colorBlending = VkPipelineColorBlendStateCreateInfo.calloc(stack);
         colorBlending.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO);
         colorBlending.logicOpEnable(false);
         colorBlending.logicOp(VK10.VK_LOGIC_OP_COPY);
         colorBlending.pAttachments(colorBlendAttachment);
         colorBlending.blendConstants(stack.floats(0.0f, 0.0f, 0.0f, 0.0f));

         // ===> PIPELINE LAYOUT CREATION <===

         VkPipelineLayoutCreateInfo pipelineLayoutInfo = VkPipelineLayoutCreateInfo.calloc(stack);
         pipelineLayoutInfo.sType(VK10.VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO);

         LongBuffer pPipelineLayout = stack.longs(VK10.VK_NULL_HANDLE);

         if (VK10.vkCreatePipelineLayout(device, pipelineLayoutInfo, null, pPipelineLayout) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create pipeline layout");
         }

         pipelineLayout = pPipelineLayout.get(0);

         // ===> RELEASE RESOURCES <===

         VK10.vkDestroyShaderModule(device, vertShaderModule, null);
         VK10.vkDestroyShaderModule(device, fragShaderModule, null);

         vertShaderSPIRV.free();
         fragShaderSPIRV.free();
      }
   }

   private long createShaderModule(ByteBuffer spirvCode)
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkShaderModuleCreateInfo createInfo = VkShaderModuleCreateInfo.calloc(stack);
         createInfo.sType(VK10.VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO);
         createInfo.pCode(spirvCode);

         LongBuffer pShaderModule = stack.mallocLong(1);
         if (VK10.vkCreateShaderModule(device, createInfo, null, pShaderModule) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create shader module");
         }
         return pShaderModule.get(0);
      }
   }

   private VkSurfaceFormatKHR chooseSwapSurfaceFormat(VkSurfaceFormatKHR.Buffer availableFormats)
   {
      return availableFormats.stream()
                             .filter(availableFormat -> availableFormat.format() == VK10.VK_FORMAT_B8G8R8_UNORM)
                             .filter(availableFormat -> availableFormat.colorSpace() == KHRSurface.VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
                             .findAny()
                             .orElse(availableFormats.get(0));
   }

   private int chooseSwapPresentMode(IntBuffer availablePresentModes)
   {
      for (int i = 0; i < availablePresentModes.capacity(); i++)
      {
         if (availablePresentModes.get(i) == KHRSurface.VK_PRESENT_MODE_MAILBOX_KHR)
         {
            return availablePresentModes.get(i);
         }
      }
      return KHRSurface.VK_PRESENT_MODE_FIFO_KHR;
   }

   private VkExtent2D chooseSwapExtent(MemoryStack stack, VkSurfaceCapabilitiesKHR capabilities)
   {
      if (capabilities.currentExtent().width() != UINT32_MAX)
      {
         return capabilities.currentExtent();
      }

      VkExtent2D actualExtent = VkExtent2D.malloc(stack).set(WIDTH, HEIGHT);
      VkExtent2D minExtent = capabilities.minImageExtent();
      VkExtent2D maxExtent = capabilities.maxImageExtent();
      actualExtent.width(MathTools.clamp(minExtent.width(), maxExtent.width(), actualExtent.width()));
      actualExtent.height(MathTools.clamp(minExtent.height(), maxExtent.height(), actualExtent.height()));
      return actualExtent;
   }

   private boolean isDeviceSuitable(VkPhysicalDevice device)
   {
      QueueFamilyIndices indices = findQueueFamilies(device);

      boolean extensionsSupported = VulkanTools.checkDeviceExtensionSupport(device);
      boolean swapChainAdequate = false;
      if (extensionsSupported)
      {
         try (MemoryStack stack = MemoryStack.stackPush())
         {
            SwapChainSupportDetails swapChainSupport = VulkanTools.querySwapChainSupport(device, stack, surface);
            swapChainAdequate = swapChainSupport.getFormats().hasRemaining() && swapChainSupport.getPresentModes().hasRemaining();
         }
      }
      return indices.isComplete() && extensionsSupported && swapChainAdequate;
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

         IntBuffer presentSupport = stack.ints(VK10.VK_FALSE);
         for (int i = 0; i < queueFamilies.capacity() || !indices.isComplete(); i++)
         {
            if ((queueFamilies.get(i).queueFlags() & VK10.VK_QUEUE_GRAPHICS_BIT) != 0)
            {
               indices.setGraphicsFamily(i);
            }

            KHRSurface.vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, presentSupport);

            if (presentSupport.get(0) == VK10.VK_TRUE)
            {
               indices.setPresentFamily(i);
            }
         }

         return indices;
      }
   }

   public static void main(String[] args)
   {
      new HelloVulkan().run();
   }
}
