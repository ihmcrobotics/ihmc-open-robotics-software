package us.ihmc.gdx;

import imgui.ImFontConfig;
import imgui.ImGuiFreeType;
import imgui.ImGuiIO;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.gdx.imgui.ImGuiGlfwWindow;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.nio.file.Files;
import java.nio.file.Paths;

public class ImGuiGlfwFreeTypeDemo
{
   private final ImGuiGlfwWindow imGuiGlfwWindow;

   private final ImFontConfig fontConfig = new ImFontConfig();
   boolean wantRebuild = true;
   float[] fontsMultiply = new float[] {1.0f};
   int[] fontsPadding = new int[] {1};
   ImInt fontsFlags = new ImInt(0);
   int[] oversampleH = new int[] {0};
   int[] oversampleV = new int[] {0};
   ImBoolean pixelSnapH = new ImBoolean(false);

   boolean addedFont = false;

   public ImGuiGlfwFreeTypeDemo()
   {
      imGuiGlfwWindow = new ImGuiGlfwWindow(getClass().getSimpleName(), 800, 600);
      imGuiGlfwWindow.getDockingSetup().addFirst("Window");
      imGuiGlfwWindow.run(this::configure, this::render, this::dispose);
   }

   public void configure()
   {
      if (wantRebuild)
      {
         wantRebuild = false;

         ImGuiIO io = ImGui.getIO();
         io.getFonts().setTexGlyphPadding(fontsPadding[0]);
         fontConfig.setPixelSnapH(pixelSnapH.get());
         fontConfig.setOversampleH(oversampleH[0]);
         fontConfig.setOversampleV(oversampleV[0]);
         fontConfig.setRasterizerMultiply(fontsMultiply[0]);
         fontConfig.setRasterizerFlags(fontsFlags.get());

         if (!addedFont)
         {
            addedFont = true;
            fontConfig.setName("Roboto-Regular.ttf, 14px");
            io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("Roboto-Regular.ttf"), 14.0f, fontConfig);
            if (Files.exists(Paths.get("/usr/share/fonts/TTF/segoeui.ttf")))
            {
               fontConfig.setName("segoeui.ttf, 16px");
               io.getFonts().addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", 16.0f, fontConfig);
            }
            fontConfig.setName("DejaVuSans.ttf, 13px");
            io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 13.0f, fontConfig);
         }

         ImGuiFreeType.buildFontAtlas(io.getFonts(), fontsFlags.get());

         imGuiGlfwWindow.getImGuiDockSystem().getImGuiGl3().updateFontsTexture();
      }
   }

   public void render()
   {

      ImGui.begin("FreeType Options");
      ImGui.text("The quick brown fox jumped over the lazy dog.");
      ImGui.showFontSelector("Fonts");
      wantRebuild |= ImGui.dragFloat("Multiply", fontsMultiply, 0.001f, 0.0f, 2.0f);
      wantRebuild |= ImGui.dragInt("Padding", fontsPadding, 0.1f, 0, 16);
      wantRebuild |= ImGui.dragInt("OversampleH", oversampleH, 0.1f, 0, 16);
      wantRebuild |= ImGui.dragInt("OversampleV", oversampleV, 0.1f, 0, 16);
      wantRebuild |= ImGui.checkbox("PixelSnapH", pixelSnapH);
      wantRebuild |= ImGui.checkboxFlags("NoHinting", fontsFlags, ImGuiFreeType.RasterizerFlags.NoHinting);
      wantRebuild |= ImGui.checkboxFlags("NoAutoHint", fontsFlags, ImGuiFreeType.RasterizerFlags.NoAutoHint);
      wantRebuild |= ImGui.checkboxFlags("ForceAutoHint", fontsFlags, ImGuiFreeType.RasterizerFlags.ForceAutoHint);
      wantRebuild |= ImGui.checkboxFlags("LightHinting", fontsFlags, ImGuiFreeType.RasterizerFlags.LightHinting);
      wantRebuild |= ImGui.checkboxFlags("MonoHinting", fontsFlags, ImGuiFreeType.RasterizerFlags.MonoHinting);
      wantRebuild |= ImGui.checkboxFlags("Bold", fontsFlags, ImGuiFreeType.RasterizerFlags.Bold);
      wantRebuild |= ImGui.checkboxFlags("Oblique", fontsFlags, ImGuiFreeType.RasterizerFlags.Oblique);
      wantRebuild |= ImGui.checkboxFlags("Monochrome", fontsFlags, ImGuiFreeType.RasterizerFlags.Monochrome);
      ImGui.end();
   }

   public void dispose()
   {
      fontConfig.destroy();
   }

   public static void main(String[] args)
   {
      new ImGuiGlfwFreeTypeDemo();
   }
}
