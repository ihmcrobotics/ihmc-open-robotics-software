package us.ihmc.gdx.imgui;

import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;

import java.nio.IntBuffer;
import java.util.Objects;

import static org.lwjgl.glfw.Callbacks.glfwFreeCallbacks;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;

public class GlfwWindowForImGui
{
   private final String windowTitle;
   private final int[] windowWidthArray = new int[] {800};
   private final int[] windowHeightArray = new int[] {600};
   private long windowHandle = NULL;

   public GlfwWindowForImGui(String windowTitle)
   {
      this.windowTitle = windowTitle;
   }

   // TODO: Extract more settings options
   public void create()
   {
      GLFWErrorCallback.createPrint(System.err).set();

      if (!glfwInit())
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }

      long monitor = NULL;
      long share = NULL;
      windowHandle = glfwCreateWindow(windowWidthArray[0], windowHeightArray[0], windowTitle, monitor, share);

      if (windowHandle == NULL) {
         throw new RuntimeException("Failed to create the GLFW window");
      }

      // Get the thread stack and push a new frame
      try (MemoryStack stack = stackPush()) {
         final IntBuffer pWidth = stack.mallocInt(1); // int*
         final IntBuffer pHeight = stack.mallocInt(1); // int*

         // Get the window size passed to glfwCreateWindow
         glfwGetWindowSize(windowHandle, pWidth, pHeight);

         // Get the resolution of the primary monitor
         final GLFWVidMode vidmode = Objects.requireNonNull(glfwGetVideoMode(glfwGetPrimaryMonitor()));

         // Center the window
         glfwSetWindowPos(windowHandle, (vidmode.width() - pWidth.get(0)) / 2, (vidmode.height() - pHeight.get(0)) / 2);
      } // the stack frame is popped automatically

      glfwMakeContextCurrent(windowHandle); // Make the OpenGL context current
      glfwSwapInterval(GLFW_TRUE); // Enable v-sync
      glfwShowWindow(windowHandle); // Make the window visible

      // IMPORTANT!!
      // This line is critical for LWJGL's interoperation with GLFW's
      // OpenGL context, or any context that is managed externally.
      // LWJGL detects the context that is current in the current thread,
      // creates the GLCapabilities instance and makes the OpenGL
      // bindings available for use.
      GL.createCapabilities();
   }

   public void dispose()
   {
      glfwFreeCallbacks(windowHandle);
      glfwDestroyWindow(windowHandle);
      glfwTerminate();
      Objects.requireNonNull(glfwSetErrorCallback(null)).free();
   }

   public long getWindowHandle()
   {
      return windowHandle;
   }

   public void setWindowSize(int width, int height)
   {
      windowWidthArray[0] = width;
      windowHeightArray[0] = height;
      if (windowHandle != NULL)
         glfwSetWindowSize(windowHandle, width, height);
   }

   public int getWindowWidth()
   {
      if (windowHandle != NULL)
         glfwGetWindowSize(windowHandle, windowWidthArray, windowHeightArray);
      return windowWidthArray[0];
   }

   public int getWindowHeight()
   {
      if (windowHandle != NULL)
         glfwGetWindowSize(windowHandle, windowWidthArray, windowHeightArray);
      return windowHeightArray[0];
   }

   public String getWindowTitle()
   {
      return windowTitle;
   }
}
