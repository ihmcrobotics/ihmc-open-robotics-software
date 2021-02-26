package us.ihmc.gdx.imgui;

import com.badlogic.gdx.Input;
import imgui.*;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.opengl.GL32;
import us.ihmc.euclid.geometry.BoundingBox2D;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Objects;

import static org.lwjgl.opengl.GL32.glClear;
import static org.lwjgl.opengl.GL32.glClearColor;

public class ImGuiTools
{
   public static float TAB_BAR_HEIGHT = 20.0f;
   public static final int GDX_TO_IMGUI_KEY_CODE_OFFSET = GLFW.GLFW_KEY_A - Input.Keys.A;

   public static void glClearDarkGray()
   {
      glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
      glClear(GL32.GL_COLOR_BUFFER_BIT);
   }

   /**
    * See https://github.com/ocornut/imgui/blob/master/docs/FONTS.md
    * and ImGuiGlfwFreeTypeDemo in this project
    */
   public static ImFont setupFonts(ImGuiIO io)
   {
      final ImFontConfig fontConfig = new ImFontConfig(); // Natively allocated object, should be explicitly destroyed

      // Glyphs could be added per-font as well as per config used globally like here
//      fontConfig.setGlyphRanges(fontAtlas.getGlyphRangesCyrillic());

//      fontConfig.setMergeMode(true); // When enabled, all fonts added with this config would be merged with the previously added font
      float size = 14.0f;
//      fontConfig.setSizePixels(size);
//      fontConfig.setOversampleH(4);
//      fontConfig.setOversampleV(4);
      int fontsFlags = 0;
      fontsFlags += ImGuiFreeType.RasterizerFlags.LightHinting;
//      fontConfig.setRasterizerFlags(flags);
//      fontConfig.setRasterizerMultiply(2.0f);
//      fontConfig.setPixelSnapH(true);

      ImFont fontToReturn;
//      fontToReturn = fontAtlas.addFontDefault(); // Add a default font, which is 'ProggyClean.ttf, 13px'
//      fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("basis33.ttf"), 16, fontConfig);
      if (Files.exists(Paths.get("/usr/share/fonts/TTF/segoeui.ttf")))
      {
         fontConfig.setName("segoeui.ttf, 16px");
         fontToReturn = io.getFonts().addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", 16.0f, fontConfig);
      }
      else
      {
         fontConfig.setName("DejaVuSans.ttf, 13px");
         fontToReturn = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 13.0f, fontConfig);
      }
//      fontConfig.setName("Roboto-Regular.ttf, 14px"); // This name will be displayed in Style Editor
//      fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), size, fontConfig);
//      fontConfig.setName("Roboto-Regular.ttf, 16px"); // We can apply a new config value every time we add a new font
//      fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), 16, fontConfig);

//      fontConfig.setName("Segoe UI"); // We can apply a new config value every time we add a new font
//      fontToReturn = fontAtlas.addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", size, fontConfig);

      ImGuiFreeType.buildFontAtlas(io.getFonts(), fontsFlags);


      fontConfig.destroy(); // After all fonts were added we don't need this config more

      return fontToReturn;
   }

   public static byte[] loadFromResources(final String fileName)
   {
      try (InputStream is = Objects.requireNonNull(ImGuiTools.class.getClassLoader().getResourceAsStream(fileName));
           ByteArrayOutputStream buffer = new ByteArrayOutputStream())
      {

         final byte[] data = new byte[16384];

         int nRead;
         while ((nRead = is.read(data, 0, data.length)) != -1)
         {
            buffer.write(data, 0, nRead);
         }

         return buffer.toByteArray();
      }
      catch (IOException e)
      {
         throw new UncheckedIOException(e);
      }
   }

   public static BoundingBox2D windowBoundingBox()
   {
      BoundingBox2D box = new BoundingBox2D();
      int posX = (int) ImGui.getWindowPosX();
      int posY = (int) ImGui.getWindowPosY();
      box.setMin(posX, posY);
      box.setMax(posX + ImGui.getWindowSizeX(), posY + (int) ImGui.getWindowSizeX());
      return box;
   }
}
