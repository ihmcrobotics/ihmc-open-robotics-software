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
   private long windowHandle;

   public void create()
   {
      GLFWErrorCallback.createPrint(System.err).set();

      if (!glfwInit())
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }

      windowHandle = glfwCreateWindow(1280, 768, "Dear ImGui+LWJGL Example", NULL, NULL);

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
}
