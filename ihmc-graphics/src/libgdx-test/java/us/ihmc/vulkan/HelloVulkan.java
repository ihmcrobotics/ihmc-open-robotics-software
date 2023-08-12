package us.ihmc.vulkan;

import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWVulkan;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import org.lwjgl.vulkan.*;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.log.LogTools;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Taken from https://github.com/Naitsirc98/Vulkan-Tutorial-Java
 */
public class HelloVulkan
{
   private static final int UINT32_MAX = 0xFFFFFFFF;
   private static final long UINT64_MAX = 0xFFFFFFFFFFFFFFFFL;
   private static final int WIDTH = 800;
   private static final int HEIGHT = 600;
   private static final int MAX_FRAMES_IN_FLIGHT = 2;

   private static final VulkanVertex[] VERTICES = {
         new VulkanVertex(new Vector2D(0.0, -0.5), new MutableColor(1.0f, 0.0f, 0.0f)),
         new VulkanVertex(new Vector2D(0.5, 0.5), new MutableColor(0.0f, 1.0f, 0.0f)),
         new VulkanVertex(new Vector2D(-0.5, 0.5), new MutableColor(0.0f, 0.0f, 1.0f))
   };

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
   private int swapChainImageFormat;
   private VkExtent2D swapChainExtent;
   private List<Long> swapChainImageViews;
   private List<Long> swapChainFramebuffers;
   private long renderPass;
   private long pipelineLayout;
   private long graphicsPipeline;
   private long commandPool;
   private long vertexBuffer;
   private long vertexBufferMemory;
   private List<VkCommandBuffer> commandBuffers;
   private List<HelloVulkanFrame> inFlightFrames;
   private Map<Integer, HelloVulkanFrame> imagesInFlight;
   private int currentFrame;
   boolean framebufferResize;

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
      GLFW.glfwWindowHint(GLFW.GLFW_RESIZABLE, GLFW.GLFW_TRUE);

      String title = HelloVulkan.class.getSimpleName();
      window = GLFW.glfwCreateWindow(WIDTH, HEIGHT, title, MemoryUtil.NULL, MemoryUtil.NULL);

      if (window == MemoryUtil.NULL)
      {
         throw new RuntimeException("Cannot create window");
      }

      GLFW.glfwSetFramebufferSizeCallback(window, this::framebufferResizeCallback);
   }

   private void framebufferResizeCallback(long window, int width, int height)
   {
      framebufferResize = true;
   }

   private void initVulkan()
   {
      createInstance();
      setupDebugMessenger();
      createSurface();
      pickPhysicalDevice();
      createLogicalDevice();
      createCommandPool();
      createVertexBuffer();
      createSwapChainObjects();
      createSyncObjects();
   }

   private void mainLoop()
   {
      while (!GLFW.glfwWindowShouldClose(window))
      {
         GLFW.glfwPollEvents();
         drawFrame();
      }

      // Wait for the device to complete all operations before release resources
      VK10.vkDeviceWaitIdle(device);
   }

   private void cleanupSwapChain()
   {
      swapChainFramebuffers.forEach(framebuffer -> VK10.vkDestroyFramebuffer(device, framebuffer, null));

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VK10.vkFreeCommandBuffers(device, commandPool, VulkanTools.asPointerBuffer(stack, commandBuffers));
      }

      VK10.vkDestroyPipeline(device, graphicsPipeline, null);
      VK10.vkDestroyPipelineLayout(device, pipelineLayout, null);
      VK10.vkDestroyRenderPass(device, renderPass, null);

      swapChainImageViews.forEach(imageView -> VK10.vkDestroyImageView(device, imageView, null));
      KHRSwapchain.vkDestroySwapchainKHR(device, swapChain, null);
   }

   private void cleanup()
   {
      cleanupSwapChain();

      VK10.vkDestroyBuffer(device, vertexBuffer, null);
      VK10.vkFreeMemory(device, vertexBufferMemory, null);

      inFlightFrames.forEach(frame ->
      {
         VK10.vkDestroySemaphore(device, frame.renderFinishedSemaphore(), null);
         VK10.vkDestroySemaphore(device, frame.imageAvailableSemaphore(), null);
         VK10.vkDestroyFence(device, frame.fence(), null);
      });
      imagesInFlight.clear();

      VK10.vkDestroyCommandPool(device, commandPool, null);

      VK10.vkDestroyDevice(device, null);

      if (VulkanTools.ENABLE_VALIDATION_LAYERS)
         VulkanTools.destroyDebugUtilsMessengerEXT(vulkanInstance, debugMessenger, null);

      KHRSurface.vkDestroySurfaceKHR(vulkanInstance, surface, null);

      VK10.vkDestroyInstance(vulkanInstance, null);

      GLFW.glfwDestroyWindow(window);
      GLFW.glfwTerminate();
   }

   private void recreateSwapChain()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         IntBuffer width = stack.ints(0);
         IntBuffer height = stack.ints(0);

         while (width.get(0) == 0 && height.get(0) == 0)
         {
            GLFW.glfwGetFramebufferSize(window, width, height);
            GLFW.glfwWaitEvents();
         }
      }

      VK10.vkDeviceWaitIdle(device);

      cleanupSwapChain();
      createSwapChainObjects();
   }

   private void createSwapChainObjects()
   {
      createSwapChain();
      createImageViews();
      createRenderPass();
      createGraphicsPipeline();
      createFramebuffers();
      createCommandBuffers();
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

   private void createRenderPass()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkAttachmentDescription.Buffer colorAttachment = VkAttachmentDescription.calloc(1, stack);
         colorAttachment.format(swapChainImageFormat);
         colorAttachment.samples(VK10.VK_SAMPLE_COUNT_1_BIT);
         colorAttachment.loadOp(VK10.VK_ATTACHMENT_LOAD_OP_CLEAR);
         colorAttachment.storeOp(VK10.VK_ATTACHMENT_STORE_OP_STORE);
         colorAttachment.stencilLoadOp(VK10.VK_ATTACHMENT_LOAD_OP_DONT_CARE);
         colorAttachment.stencilStoreOp(VK10.VK_ATTACHMENT_STORE_OP_DONT_CARE);
         colorAttachment.initialLayout(VK10.VK_IMAGE_LAYOUT_UNDEFINED);
         colorAttachment.finalLayout(KHRSwapchain.VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

         VkAttachmentReference.Buffer colorAttachmentReference = VkAttachmentReference.calloc(1, stack);
         colorAttachmentReference.attachment(0);
         colorAttachmentReference.layout(VK10.VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

         VkSubpassDescription.Buffer subpass = VkSubpassDescription.calloc(1, stack);
         subpass.pipelineBindPoint(VK10.VK_PIPELINE_BIND_POINT_GRAPHICS);
         subpass.colorAttachmentCount(1);
         subpass.pColorAttachments(colorAttachmentReference);

         VkSubpassDependency.Buffer dependency = VkSubpassDependency.calloc(1, stack);
         dependency.srcSubpass(VK10.VK_SUBPASS_EXTERNAL);
         dependency.dstSubpass(0);
         dependency.srcStageMask(VK10.VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
         dependency.srcAccessMask(0);
         dependency.dstStageMask(VK10.VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
         dependency.dstAccessMask(VK10.VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK10.VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT);

         VkRenderPassCreateInfo renderPassInfo = VkRenderPassCreateInfo.calloc(stack);
         renderPassInfo.sType(VK10.VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO);
         renderPassInfo.pAttachments(colorAttachment);
         renderPassInfo.pSubpasses(subpass);
         renderPassInfo.pDependencies(dependency);

         LongBuffer pRenderPass = stack.mallocLong(1);

         if (VK10.vkCreateRenderPass(device, renderPassInfo, null, pRenderPass) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create render pass");
         }

         renderPass = pRenderPass.get(0);
      }
   }

   private void createGraphicsPipeline()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         // Let's compile the GLSL shaders into SPIR-V at runtime using the shaderc library
         // Check ShaderSPIRVUtils class to see how it can be done
         ShaderSPIRVUtils.SPIRV vertShaderSPIRV
               = ShaderSPIRVUtils.compileShaderFile("shaders/17_shader_vertexbuffer.vert", ShaderSPIRVUtils.ShaderKind.VERTEX_SHADER);
         ShaderSPIRVUtils.SPIRV fragShaderSPIRV
               = ShaderSPIRVUtils.compileShaderFile("shaders/17_shader_vertexbuffer.frag", ShaderSPIRVUtils.ShaderKind.FRAGMENT_SHADER);
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
         vertexInputInfo.pVertexBindingDescriptions(VulkanVertex.getBindingDescription(stack));
         vertexInputInfo.pVertexAttributeDescriptions(VulkanVertex.getAttributeDescriptions(stack));

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

         VkGraphicsPipelineCreateInfo.Buffer pipelineInfo = VkGraphicsPipelineCreateInfo.calloc(1, stack);
         pipelineInfo.sType(VK10.VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO);
         pipelineInfo.pStages(shaderStages);
         pipelineInfo.pVertexInputState(vertexInputInfo);
         pipelineInfo.pInputAssemblyState(inputAssembly);
         pipelineInfo.pViewportState(viewportState);
         pipelineInfo.pRasterizationState(rasterizer);
         pipelineInfo.pMultisampleState(multisampling);
         pipelineInfo.pColorBlendState(colorBlending);
         pipelineInfo.layout(pipelineLayout);
         pipelineInfo.renderPass(renderPass);
         pipelineInfo.subpass(0);
         pipelineInfo.basePipelineHandle(VK10.VK_NULL_HANDLE);
         pipelineInfo.basePipelineIndex(-1);

         LongBuffer pGraphicsPipeline = stack.mallocLong(1);
         if (VK10.vkCreateGraphicsPipelines(device, VK10.VK_NULL_HANDLE, pipelineInfo, null, pGraphicsPipeline) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create graphics pipeline");
         }
         graphicsPipeline = pGraphicsPipeline.get(0);

         // ===> RELEASE RESOURCES <===

         VK10.vkDestroyShaderModule(device, vertShaderModule, null);
         VK10.vkDestroyShaderModule(device, fragShaderModule, null);

         vertShaderSPIRV.free();
         fragShaderSPIRV.free();
      }
   }

   private void createFramebuffers()
   {
      swapChainFramebuffers = new ArrayList<>(swapChainImageViews.size());

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         LongBuffer attachments = stack.mallocLong(1);
         LongBuffer pFramebuffer = stack.mallocLong(1);

         // Lets allocate the create info struct once and just update the pAttachments field each iteration
         VkFramebufferCreateInfo framebufferInfo = VkFramebufferCreateInfo.calloc(stack);
         framebufferInfo.sType(VK10.VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO);
         framebufferInfo.renderPass(renderPass);
         framebufferInfo.width(swapChainExtent.width());
         framebufferInfo.height(swapChainExtent.height());
         framebufferInfo.layers(1);

         for (long imageView : swapChainImageViews)
         {
            attachments.put(0, imageView);
            framebufferInfo.pAttachments(attachments);

            if (VK10.vkCreateFramebuffer(device, framebufferInfo, null, pFramebuffer) != VK10.VK_SUCCESS)
            {
               throw new RuntimeException("Failed to create framebuffer");
            }

            swapChainFramebuffers.add(pFramebuffer.get(0));
         }
      }
   }

   private void createCommandPool()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

         VkCommandPoolCreateInfo poolInfo = VkCommandPoolCreateInfo.calloc(stack);
         poolInfo.sType(VK10.VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO);
         poolInfo.queueFamilyIndex(queueFamilyIndices.getGraphicsFamily());

         LongBuffer pCommandPool = stack.mallocLong(1);
         if (VK10.vkCreateCommandPool(device, poolInfo, null, pCommandPool) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create command pool");
         }
         commandPool = pCommandPool.get(0);
      }
   }

   private void createVertexBuffer()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         long bufferSize = VulkanVertex.SIZEOF * VERTICES.length;

         LongBuffer pBuffer = stack.mallocLong(1);
         LongBuffer pBufferMemory = stack.mallocLong(1);
         createBuffer(bufferSize,
                      VK10.VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                      VK10.VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK10.VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      pBuffer,
                      pBufferMemory);

         long stagingBuffer = pBuffer.get(0);
         long stagingBufferMemory = pBufferMemory.get(0);

         PointerBuffer data = stack.mallocPointer(1);

         VK10.vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, data);
         {
            memcpy(data.getByteBuffer(0, (int) bufferSize), VERTICES);
         }
         VK10.vkUnmapMemory(device, stagingBufferMemory);

         createBuffer(bufferSize,
                      VK10.VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK10.VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                      VK10.VK_MEMORY_HEAP_DEVICE_LOCAL_BIT,
                      pBuffer,
                      pBufferMemory);

         vertexBuffer = pBuffer.get(0);
         vertexBufferMemory = pBufferMemory.get(0);

         copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

         VK10.vkDestroyBuffer(device, stagingBuffer, null);
         VK10.vkFreeMemory(device, stagingBufferMemory, null);
      }
   }

   private void createBuffer(long size, int usage, int properties, LongBuffer pBuffer, LongBuffer pBufferMemory)
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkBufferCreateInfo bufferInfo = VkBufferCreateInfo.calloc(stack);
         bufferInfo.sType(VK10.VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
         bufferInfo.size(size);
         bufferInfo.usage(usage);
         bufferInfo.sharingMode(VK10.VK_SHARING_MODE_EXCLUSIVE);

         if (VK10.vkCreateBuffer(device, bufferInfo, null, pBuffer) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to create vertex buffer");
         }

         VkMemoryRequirements memRequirements = VkMemoryRequirements.malloc(stack);
         VK10.vkGetBufferMemoryRequirements(device, pBuffer.get(0), memRequirements);

         VkMemoryAllocateInfo allocInfo = VkMemoryAllocateInfo.calloc(stack);
         allocInfo.sType(VK10.VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO);
         allocInfo.allocationSize(memRequirements.size());
         allocInfo.memoryTypeIndex(findMemoryType(stack, memRequirements.memoryTypeBits(), properties));

         if (VK10.vkAllocateMemory(device, allocInfo, null, pBufferMemory) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to allocate vertex buffer memory");
         }

         VK10.vkBindBufferMemory(device, pBuffer.get(0), pBufferMemory.get(0), 0);
      }
   }

   private void copyBuffer(long srcBuffer, long dstBuffer, long size)
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkCommandBufferAllocateInfo allocInfo = VkCommandBufferAllocateInfo.calloc(stack);
         allocInfo.sType(VK10.VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
         allocInfo.level(VK10.VK_COMMAND_BUFFER_LEVEL_PRIMARY);
         allocInfo.commandPool(commandPool);
         allocInfo.commandBufferCount(1);

         PointerBuffer pCommandBuffer = stack.mallocPointer(1);
         VK10.vkAllocateCommandBuffers(device, allocInfo, pCommandBuffer);
         VkCommandBuffer commandBuffer = new VkCommandBuffer(pCommandBuffer.get(0), device);

         VkCommandBufferBeginInfo beginInfo = VkCommandBufferBeginInfo.calloc(stack);
         beginInfo.sType(VK10.VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
         beginInfo.flags(VK10.VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

         VK10.vkBeginCommandBuffer(commandBuffer, beginInfo);
         {
            VkBufferCopy.Buffer copyRegion = VkBufferCopy.calloc(1, stack);
            copyRegion.size(size);
            VK10.vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, copyRegion);
         }
         VK10.vkEndCommandBuffer(commandBuffer);

         VkSubmitInfo submitInfo = VkSubmitInfo.calloc(stack);
         submitInfo.sType(VK10.VK_STRUCTURE_TYPE_SUBMIT_INFO);
         submitInfo.pCommandBuffers(pCommandBuffer);

         if (VK10.vkQueueSubmit(graphicsQueue, submitInfo, VK10.VK_NULL_HANDLE) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to submit copy command buffer");
         }

         VK10.vkQueueWaitIdle(graphicsQueue);

         VK10.vkFreeCommandBuffers(device, commandPool, pCommandBuffer);
      }
   }

   private void memcpy(ByteBuffer buffer, VulkanVertex[] vertices)
   {
      for (VulkanVertex vertex : vertices)
      {
         buffer.putFloat(vertex.getPosition().getX32());
         buffer.putFloat(vertex.getPosition().getY32());

         buffer.putFloat(vertex.getColor().getX());
         buffer.putFloat(vertex.getColor().getY());
         buffer.putFloat(vertex.getColor().getZ());
      }
   }

   private int findMemoryType(MemoryStack stack, int typeFilter, int properties)
   {
      VkPhysicalDeviceMemoryProperties memProperties = VkPhysicalDeviceMemoryProperties.malloc(stack);
      VK10.vkGetPhysicalDeviceMemoryProperties(physicalDevice, memProperties);

      for (int i = 0; i < memProperties.memoryTypeCount(); i++)
      {
         if ((typeFilter & (1 << i)) != 0 && (memProperties.memoryTypes(i).propertyFlags() & properties) == properties)
         {
            return i;
         }
      }

      throw new RuntimeException("Failed to find suitable memory type");
   }

   private void createCommandBuffers()
   {
      final int commandBuffersCount = swapChainFramebuffers.size();
      commandBuffers = new ArrayList<>(commandBuffersCount);

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkCommandBufferAllocateInfo allocInfo = VkCommandBufferAllocateInfo.calloc(stack);
         allocInfo.sType(VK10.VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
         allocInfo.commandPool(commandPool);
         allocInfo.level(VK10.VK_COMMAND_BUFFER_LEVEL_PRIMARY);
         allocInfo.commandBufferCount(commandBuffersCount);

         PointerBuffer pCommandBuffers = stack.mallocPointer(commandBuffersCount);

         if (VK10.vkAllocateCommandBuffers(device, allocInfo, pCommandBuffers) != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to allocate command buffers");
         }

         for (int i = 0; i < commandBuffersCount; i++)
         {
            commandBuffers.add(new VkCommandBuffer(pCommandBuffers.get(i), device));
         }

         VkCommandBufferBeginInfo beginInfo = VkCommandBufferBeginInfo.calloc(stack);
         beginInfo.sType(VK10.VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);

         VkRenderPassBeginInfo renderPassInfo = VkRenderPassBeginInfo.calloc(stack);
         renderPassInfo.sType(VK10.VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO);
         renderPassInfo.renderPass(renderPass);
         VkRect2D renderArea = VkRect2D.calloc(stack);
         renderArea.offset(VkOffset2D.calloc(stack).set(0, 0));
         renderArea.extent(swapChainExtent);
         renderPassInfo.renderArea(renderArea);
         VkClearValue.Buffer clearValues = VkClearValue.calloc(1, stack);
         clearValues.color().float32(stack.floats(0.0f, 0.0f, 0.0f, 1.0f));
         renderPassInfo.pClearValues(clearValues);

         for (int i = 0; i < commandBuffersCount; i++)
         {
            VkCommandBuffer commandBuffer = commandBuffers.get(i);
            if (VK10.vkBeginCommandBuffer(commandBuffer, beginInfo) != VK10.VK_SUCCESS)
            {
               throw new RuntimeException("Failed to begin recording command buffer");
            }

            renderPassInfo.framebuffer(swapChainFramebuffers.get(i));

            VK10.vkCmdBeginRenderPass(commandBuffer, renderPassInfo, VK10.VK_SUBPASS_CONTENTS_INLINE);
            {
               VK10.vkCmdBindPipeline(commandBuffer, VK10.VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);

               LongBuffer vertexBuffers = stack.longs(vertexBuffer);
               LongBuffer offsets = stack.longs(0);
               VK10.vkCmdBindVertexBuffers(commandBuffer, 0, vertexBuffers, offsets);

               VK10.vkCmdDraw(commandBuffer, VERTICES.length, 1, 0, 0);
            }
            VK10.vkCmdEndRenderPass(commandBuffer);

            if (VK10.vkEndCommandBuffer(commandBuffer) != VK10.VK_SUCCESS)
            {
               throw new RuntimeException("Failed to record command buffer");
            }
         }
      }
   }

   private void createSyncObjects()
   {
      inFlightFrames = new ArrayList<>(MAX_FRAMES_IN_FLIGHT);
      imagesInFlight = new HashMap<>(swapChainImages.size());

      try (MemoryStack stack = MemoryStack.stackPush())
      {
         VkSemaphoreCreateInfo semaphoreInfo = VkSemaphoreCreateInfo.calloc(stack);
         semaphoreInfo.sType(VK10.VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO);

         VkFenceCreateInfo fenceInfo = VkFenceCreateInfo.calloc(stack);
         fenceInfo.sType(VK10.VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
         fenceInfo.flags(VK10.VK_FENCE_CREATE_SIGNALED_BIT);

         LongBuffer pImageAvailableSemaphore = stack.mallocLong(1);
         LongBuffer pRenderFinishedSemaphore = stack.mallocLong(1);
         LongBuffer pFence = stack.mallocLong(1);

         for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
         {
            if (VK10.vkCreateSemaphore(device, semaphoreInfo, null, pImageAvailableSemaphore) != VK10.VK_SUCCESS
             || VK10.vkCreateSemaphore(device, semaphoreInfo, null, pRenderFinishedSemaphore) != VK10.VK_SUCCESS
             || VK10.vkCreateFence(device, fenceInfo, null, pFence) != VK10.VK_SUCCESS)
            {

               throw new RuntimeException("Failed to create synchronization objects for the frame " + i);
            }

            inFlightFrames.add(new HelloVulkanFrame(pImageAvailableSemaphore.get(0), pRenderFinishedSemaphore.get(0), pFence.get(0)));
         }
      }
   }

   private void drawFrame()
   {
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         HelloVulkanFrame thisFrame = inFlightFrames.get(currentFrame);

         VK10.vkWaitForFences(device, thisFrame.pFence(), true, UINT64_MAX);

         IntBuffer pImageIndex = stack.mallocInt(1);

         int vkResult = KHRSwapchain.vkAcquireNextImageKHR(device,
                                                           swapChain,
                                                           UINT64_MAX,
                                                           thisFrame.imageAvailableSemaphore(),
                                                           VK10.VK_NULL_HANDLE,
                                                           pImageIndex);
         if (vkResult == KHRSwapchain.VK_ERROR_OUT_OF_DATE_KHR)
         {
            recreateSwapChain();
            return;
         }
         else if (vkResult != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Cannot get image");
         }

         final int imageIndex = pImageIndex.get(0);

         if (imagesInFlight.containsKey(imageIndex))
         {
            VK10.vkWaitForFences(device, imagesInFlight.get(imageIndex).fence(), true, UINT64_MAX);
         }

         imagesInFlight.put(imageIndex, thisFrame);

         VkSubmitInfo submitInfo = VkSubmitInfo.calloc(stack);
         submitInfo.sType(VK10.VK_STRUCTURE_TYPE_SUBMIT_INFO);

         submitInfo.waitSemaphoreCount(1);
         submitInfo.pWaitSemaphores(thisFrame.pImageAvailableSemaphore());
         submitInfo.pWaitDstStageMask(stack.ints(VK10.VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT));

         submitInfo.pSignalSemaphores(thisFrame.pRenderFinishedSemaphore());

         submitInfo.pCommandBuffers(stack.pointers(commandBuffers.get(imageIndex)));

         VK10.vkResetFences(device, thisFrame.pFence());

         if ((vkResult = VK10.vkQueueSubmit(graphicsQueue, submitInfo, thisFrame.fence())) != VK10.VK_SUCCESS)
         {
            VK10.vkResetFences(device, thisFrame.pFence());
            throw new RuntimeException("Failed to submit draw command buffer: " + vkResult);
         }

         VkPresentInfoKHR presentInfo = VkPresentInfoKHR.calloc(stack);
         presentInfo.sType(KHRSwapchain.VK_STRUCTURE_TYPE_PRESENT_INFO_KHR);

         presentInfo.pWaitSemaphores(thisFrame.pRenderFinishedSemaphore());

         presentInfo.swapchainCount(1);
         presentInfo.pSwapchains(stack.longs(swapChain));

         presentInfo.pImageIndices(pImageIndex);

         vkResult = KHRSwapchain.vkQueuePresentKHR(presentQueue, presentInfo);

         if (vkResult == KHRSwapchain.VK_ERROR_OUT_OF_DATE_KHR || vkResult == KHRSwapchain.VK_SUBOPTIMAL_KHR || framebufferResize)
         {
            framebufferResize = false;
            recreateSwapChain();
         }
         else if (vkResult != VK10.VK_SUCCESS)
         {
            throw new RuntimeException("Failed to present swap chain image");
         }

         currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
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

      IntBuffer width = stack.ints(0);
      IntBuffer height = stack.ints(0);
      GLFW.glfwGetFramebufferSize(window, width, height);
      VkExtent2D actualExtent = VkExtent2D.malloc(stack).set(width.get(0), height.get(0));
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
