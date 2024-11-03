package us.ihmc.rdx.imgui;

import com.google.common.io.ByteStreams;
import org.apache.commons.lang3.SystemUtils;
import org.lwjgl.glfw.*;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.io.resources.ResourceTools;
import us.ihmc.tools.processManagement.UnsignedByteTools;
import us.ihmc.tools.thread.Throttler;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.color.ColorSpace;
import java.awt.image.*;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.file.Paths;
import java.util.Hashtable;
import java.util.Objects;

public class GlfwWindowForImGui
{
   private final String windowTitle;
   private final int[] windowWidthArray = new int[] {800};
   private final int[] windowHeightArray = new int[] {600};
   private long windowHandle = MemoryUtil.NULL;
   private final Throttler frameRateLimiter = new Throttler();
   private double minFramePeriod = UnitConversions.hertzToSeconds(240);

   public GlfwWindowForImGui(String windowTitle)
   {
      this.windowTitle = windowTitle;
   }

   // TODO: Extract more settings options
   public void create()
   {
      GLFWErrorCallback.createPrint(System.err).set();

      if (!GLFW.glfwInit())
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }

      long monitor = MemoryUtil.NULL;
      long share = MemoryUtil.NULL;
      windowHandle = GLFW.glfwCreateWindow(windowWidthArray[0], windowHeightArray[0], windowTitle, monitor, share);

      if (windowHandle == MemoryUtil.NULL) {
         throw new RuntimeException("Failed to create the GLFW window");
      }

      // Get the thread stack and push a new frame
      try (MemoryStack stack = MemoryStack.stackPush())
      {
         final IntBuffer pWidth = stack.mallocInt(1); // int*
         final IntBuffer pHeight = stack.mallocInt(1); // int*

         // Get the window size passed to glfwCreateWindow
         GLFW.glfwGetWindowSize(windowHandle, pWidth, pHeight);

         // Get the resolution of the primary monitor
         final GLFWVidMode vidmode = Objects.requireNonNull(GLFW.glfwGetVideoMode(GLFW.glfwGetPrimaryMonitor()));

         // Center the window
         GLFW.glfwSetWindowPos(windowHandle, (vidmode.width() - pWidth.get(0)) / 2, (vidmode.height() - pHeight.get(0)) / 2);
      } // the stack frame is popped automatically

      GLFW.glfwMakeContextCurrent(windowHandle); // Make the OpenGL context current
      setVSyncEnabled(true);
      GLFW.glfwShowWindow(windowHandle); // Make the window visible

      // IMPORTANT!!
      // This line is critical for LWJGL's interoperation with GLFW's
      // OpenGL context, or any context that is managed externally.
      // LWJGL detects the context that is current in the current thread,
      // creates the GLCapabilities instance and makes the OpenGL
      // bindings available for use.
      GL.createCapabilities();
   }

   public void launch(Runnable render)
   {
      while (!GLFW.glfwWindowShouldClose(windowHandle))
      {
         render.run();

         frameRateLimiter.waitAndRun(minFramePeriod);

         GLFW.glfwSwapBuffers(windowHandle);
         GLFW.glfwPollEvents();
      }
   }

   public void setIcon(String... imagePaths)
   {
      if (SystemUtils.IS_OS_MAC)
         return;

      GLFWImage.Buffer glfwImageBuffer = GLFWImage.malloc(imagePaths.length);

      for (String imagePath : imagePaths)
      {
         InputStream imageInputStream = ResourceTools.openStreamSystem(Paths.get(imagePath));
         BufferedImage bufferedImage = ExceptionTools.handle(() -> ImageIO.read(imageInputStream),
                                                             DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

         int width = bufferedImage.getWidth();
         int height = bufferedImage.getHeight();
         int[] pixels = new int[width * height];
         bufferedImage.getRGB(0, 0, width, height, pixels, 0, width);

         byte[] manAssArr = new byte[width * height * 4];
//         for (int i = 0; i < 5; i++)
//         {
//            LogTools.info(StringTools.format("RGBA0: {} {} {} {}",
//                                             ((pixels[i] & 0xff000000) >>> 24) / 255f,
//                                             ((pixels[i] & 0x00ff0000) >>> 16) / 255f,
//                                             ((pixels[i] & 0x0000ff00) >>> 8) / 255f,
//                                             ((pixels[i] & 0x000000ff)) / 255f));
//         }
         int byteArrIndex = 0;
         for (int i = 0; i < pixels.length; i++)
         {
            manAssArr[byteArrIndex] = UnsignedByteTools.fromInt(pixels[i] & 0x000000ff);
            byteArrIndex++;
            manAssArr[byteArrIndex] = UnsignedByteTools.fromInt((pixels[i] & 0x0000ff00) >>> 8);
            byteArrIndex++;
            manAssArr[byteArrIndex] = UnsignedByteTools.fromInt((pixels[i] & 0x00ff0000) >>> 16);
            byteArrIndex++;
            manAssArr[byteArrIndex] = UnsignedByteTools.fromInt((pixels[i] & 0xff000000) >>> 24);
            byteArrIndex++;
         }

         boolean hasAlpha = true;
         boolean isAlphaPremultiplied = false;
         ComponentColorModel colorModel = new ComponentColorModel(ColorSpace.getInstance(ColorSpace.CS_sRGB),
                                                                  hasAlpha,
                                                                  isAlphaPremultiplied,
                                                                  Transparency.TRANSLUCENT,
                                                                  DataBuffer.TYPE_BYTE);
         int scanlineStride = width * 4;
         int pixelStride = 4;
         int[] bandOffsets = {0, 1, 2, 3};
         Point location = null;
         WritableRaster raster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE,
                                                                width,
                                                                height,
                                                                scanlineStride,
                                                                pixelStride,
                                                                bandOffsets,
                                                                location); // R, G, B, A order
         Hashtable<?, ?> properties = null;
         BufferedImage imageToo = new BufferedImage(colorModel, raster, colorModel.isAlphaPremultiplied(), properties);

         // This array will be in the same R, G, B, A order
         byte[] rgbaTwo = ((DataBufferByte) raster.getDataBuffer()).getData();

         // Draw the image onto the RGBA buffer, which will be updated immediately
         Graphics2D g = imageToo.createGraphics();
         try {
            g.setComposite(AlphaComposite.Src);
            g.drawImage(bufferedImage, 0, 0, null);
         }
         finally {
            g.dispose();
         }

//         for (int i = 0; i < 5; i++)
//         {
//            LogTools.info(StringTools.format("RGBA: {} {} {} {}",
//                                             Byte.toUnsignedInt(rgbaTwo[i * 4]),
//                                             Byte.toUnsignedInt(rgbaTwo[i * 4 + 1]),
//                                             Byte.toUnsignedInt(rgbaTwo[i * 4 + 2]),
//                                             Byte.toUnsignedInt(rgbaTwo[i * 4 + 3])));
//         }

         InputStream imageInputStream2 = ResourceTools.openStreamSystem(Paths.get(imagePath));

         byte[] byteArray = ExceptionTools.handle(() -> ByteStreams.toByteArray(imageInputStream2),
                                                  DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

         ByteBuffer byteBuffer = ByteBuffer.allocateDirect(rgbaTwo.length);




         byteBuffer.put(rgbaTwo);

//         for (int i = 0; i < 50; i++)
//         {
//            LogTools.info(StringTools.format("RGBA2: {} {} {} {}",
//                                             Byte.toUnsignedInt(manAssArr[i * 4]),
//                                             Byte.toUnsignedInt(manAssArr[i * 4 + 1]),
//                                             Byte.toUnsignedInt(manAssArr[i * 4 + 2]),
//                                             Byte.toUnsignedInt(manAssArr[i * 4 + 3])));
//         }
//         if (imagePath.contains("16"))
//         {
//            width = height = 16;
//         }
//         else if (imagePath.contains("32"))
//         {
//            width = height = 32;
//         }
//         else // if (imagePath.contains("48"))
//         {
//            width = height = 48;
//         }


         ByteBuffer byteBuffer2 = ByteBuffer.allocateDirect(width * height * 4);
         byteBuffer2.put(manAssArr);

         GLFWImage icon = GLFWImage.malloc();
         icon.set(width, height, byteBuffer2);
         glfwImageBuffer.put(icon);
         icon.free();
      }

      glfwImageBuffer.position(0);
      GLFW.glfwSetWindowIcon(windowHandle, glfwImageBuffer);
      glfwImageBuffer.free();

      //////////////////////


//      Files.FileType imageFileType = Files.FileType.Internal;
//
//      Pixmap[] pixmaps = new Pixmap[imagePaths.length];
//      Lwjgl3Files files = new Lwjgl3Files();
//      for (int i = 0; i < imagePaths.length; i++) {
//         FileHandle fileHandle = files.getFileHandle(imagePaths[i], imageFileType);
//         pixmaps[i] = new Pixmap(fileHandle);
//      }
//
//      GLFWImage.Buffer buffer = GLFWImage.malloc(pixmaps.length);
//      Pixmap[] tmpPixmaps = new Pixmap[pixmaps.length];
//
//      for (int i = 0; i < pixmaps.length; i++)
//      {
//         Pixmap pixmap = pixmaps[i];
//
//         if (pixmap.getFormat() != Pixmap.Format.RGBA8888)
//         {
//            Pixmap rgba = new Pixmap(pixmap.getWidth(), pixmap.getHeight(), Pixmap.Format.RGBA8888);
//            rgba.setBlending(Pixmap.Blending.None);
//            rgba.drawPixmap(pixmap, 0, 0);
//            tmpPixmaps[i] = rgba;
//            pixmap = rgba;
//         }
//
//         GLFWImage icon = GLFWImage.malloc();
//         icon.set(pixmap.getWidth(), pixmap.getHeight(), pixmap.getPixels());
//         buffer.put(icon);
//
//         icon.free();
//      }
//
//      buffer.position(0);
//      org.lwjgl.glfw.GLFW.glfwSetWindowIcon(windowHandle, buffer);
//
//      buffer.free();
//      for (Pixmap pixmap : tmpPixmaps)
//      {
//         if (pixmap != null)
//         {
//            pixmap.dispose();
//         }
//      }
//
//      for (Pixmap pixmap : pixmaps) {
//         pixmap.dispose();
//      }

      //      ByteBuffer byteBuffer = ByteBuffer.allocateDirect(10);
      //      GLFWImage.Buffer glfwImages = new GLFWImage.Buffer(byteBuffer);
      //      glfwSetWindowIcon(windowHandle, glfwImages);
   }

   public void dispose()
   {
      Callbacks.glfwFreeCallbacks(windowHandle);
      GLFW.glfwDestroyWindow(windowHandle);
      GLFW.glfwTerminate();
      Objects.requireNonNull(GLFW.glfwSetErrorCallback(null)).free();
   }

   public void setVSyncEnabled(boolean vSyncEnabled)
   {
      // Technically takes more values that 0 or 1, but 0 and 1 correspond to basically vsync
      GLFW.glfwSwapInterval(vSyncEnabled ? GLFW.GLFW_TRUE : GLFW.GLFW_FALSE);
   }

   public void setMaxFrameRate(double maxFrameRate)
   {
      minFramePeriod = UnitConversions.hertzToSeconds(maxFrameRate);
   }

   public long getWindowHandle()
   {
      return windowHandle;
   }

   public void setWindowSize(int width, int height)
   {
      windowWidthArray[0] = width;
      windowHeightArray[0] = height;
      if (windowHandle != MemoryUtil.NULL)
         GLFW.glfwSetWindowSize(windowHandle, width, height);
   }

   public int getWindowWidth()
   {
      if (windowHandle != MemoryUtil.NULL)
      {
         GLFW.glfwGetWindowSize(windowHandle, windowWidthArray, windowHeightArray);
      }
      return windowWidthArray[0];
   }

   public int getWindowHeight()
   {
      if (windowHandle != MemoryUtil.NULL)
      {
         GLFW.glfwGetWindowSize(windowHandle, windowWidthArray, windowHeightArray);
      }
      return windowHeightArray[0];
   }

   public String getWindowTitle()
   {
      return windowTitle;
   }
}
