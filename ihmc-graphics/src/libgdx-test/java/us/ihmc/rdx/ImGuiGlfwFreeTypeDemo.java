package us.ihmc.rdx;

import imgui.ImFontAtlas;
import imgui.ImFontConfig;
import imgui.ImGuiIO;
import imgui.flag.ImGuiFreeTypeBuilderFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.rdx.imgui.ImGuiGlfwWindow;
import us.ihmc.rdx.imgui.ImGuiTools;

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
      imGuiGlfwWindow = new ImGuiGlfwWindow(getClass());
      imGuiGlfwWindow.getPanelManager().addSelfManagedPanel("Window");
      imGuiGlfwWindow.run(null, this::configure, this::render, this::dispose);

      fontsFlags.set(fontsFlags.get() + ImGuiFreeTypeBuilderFlags.LightHinting);
   }

   public void configure()
   {
      if (wantRebuild)
      {
         wantRebuild = false;

         ImGuiIO io = ImGui.getIO();
         ImFontAtlas atlas = io.getFonts();

         // TODO: Figure out how to rebuild the fonts. This app doesn't work because the fonts get created once.

         atlas.setTexGlyphPadding(fontsPadding[0]);
         fontConfig.setPixelSnapH(pixelSnapH.get());
         fontConfig.setOversampleH(oversampleH[0]);
         fontConfig.setOversampleV(oversampleV[0]);
         fontConfig.setRasterizerMultiply(fontsMultiply[0]);
         fontConfig.setFontBuilderFlags(fontsFlags.get());

         if (!addedFont)
         {
            addedFont = true;
            fontConfig.setName("Roboto-Regular.ttf, 14px");
            atlas.addFontFromMemoryTTF(ImGuiTools.loadFromResources("Roboto-Regular.ttf"), 14.0f, fontConfig);
            if (Files.exists(Paths.get("/usr/share/fonts/TTF/segoeui.ttf")))
            {
               fontConfig.setName("segoeui.ttf, 16px");
               atlas.addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", 16.0f, fontConfig);
            }
            fontConfig.setName("DejaVuSans.ttf, 13px");
            atlas.addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 13.0f, fontConfig);
         }

         atlas.build();

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
      wantRebuild |= ImGui.checkboxFlags("NoHinting", fontsFlags, ImGuiFreeTypeBuilderFlags.NoHinting);
      wantRebuild |= ImGui.checkboxFlags("NoAutoHint", fontsFlags, ImGuiFreeTypeBuilderFlags.NoAutoHint);
      wantRebuild |= ImGui.checkboxFlags("ForceAutoHint", fontsFlags, ImGuiFreeTypeBuilderFlags.ForceAutoHint);
      wantRebuild |= ImGui.checkboxFlags("LightHinting", fontsFlags, ImGuiFreeTypeBuilderFlags.LightHinting);
      wantRebuild |= ImGui.checkboxFlags("MonoHinting", fontsFlags, ImGuiFreeTypeBuilderFlags.MonoHinting);
      wantRebuild |= ImGui.checkboxFlags("Bold", fontsFlags, ImGuiFreeTypeBuilderFlags.Bold);
      wantRebuild |= ImGui.checkboxFlags("Oblique", fontsFlags, ImGuiFreeTypeBuilderFlags.Oblique);
      wantRebuild |= ImGui.checkboxFlags("Monochrome", fontsFlags, ImGuiFreeTypeBuilderFlags.Monochrome);
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
