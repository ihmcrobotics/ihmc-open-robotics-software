package us.ihmc.gdx.imgui;

import imgui.ImFont;
import imgui.ImGuiIO;
import imgui.ImGuiStyle;
import imgui.flag.*;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGui;
import imgui.type.ImString;
import org.lwjgl.glfw.GLFWErrorCallback;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.TreeSet;

import static org.lwjgl.glfw.GLFW.*;

public class GDXImGuiWindowAndDockSystem
{
   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();

   private String glslVersion; // TODO: ?
   private long windowHandle;
   private ImFont imFont;
   private int dockspaceId;
   private final ImString newDockPanelName = new ImString("", 100);
   private final TreeSet<ImGuiDockspacePanel> dockPanelSet = new TreeSet<>(Comparator.comparing(ImGuiDockspacePanel::getName));

   private final HybridFile imGuiSettingsFile;

   public GDXImGuiWindowAndDockSystem(HybridDirectory configurationDirectory)
   {
      imGuiSettingsFile = new HybridFile(configurationDirectory, "ImGuiSettings.ini");
   }

   public void create(long windowHandle)
   {
      this.windowHandle = windowHandle;

      GLFWErrorCallback.createPrint(System.err).set();

      if (!glfwInit())
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }
//               glfwDefaultWindowHints();
//               if (SystemUtils.IS_OS_MAC) {
//                  glslVersion = "#version 150";
//                  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//                  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
//                  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
//                  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL20.GL_TRUE);            // Required on Mac
//               } else {
//                  glslVersion = "#version 130";
//                  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//                  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
//               }
//
//               GL.createCapabilities();

      ImGui.createContext();

      final ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
//               io.addConfigFlags(ImGuiConfigFlags.NavEnableKeyboard);
      io.addConfigFlags(ImGuiConfigFlags.DockingEnable);
      io.addConfigFlags(ImGuiConfigFlags.ViewportsEnable);
      io.setConfigViewportsNoTaskBarIcon(true);
      io.setConfigWindowsMoveFromTitleBarOnly(true);
      io.setConfigViewportsNoDecoration(false);
      io.setConfigDockingTransparentPayload(false);

      if (!Boolean.parseBoolean(System.getProperty("imgui.dark")))
         ImGui.styleColorsLight();
      imFont = ImGuiTools.setupFonts(io);

      // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
      if (io.hasConfigFlags(ImGuiConfigFlags.ViewportsEnable))
      {
         final ImGuiStyle style = imgui.ImGui.getStyle();
         style.setWindowRounding(0.0f);
         style.setColor(ImGuiCol.WindowBg, imgui.ImGui.getColorU32(ImGuiCol.WindowBg, 1));
      }

      imGuiGlfw.init(windowHandle, true);
      imGuiGl3.init(glslVersion);
   }

   public void beforeWindowManagement(boolean isFirstRenderCall)
   {
      if (isFirstRenderCall)
      {
         loadSettings(true);
      }

      imGuiGlfw.newFrame();
      ImGui.newFrame();

      ImGui.pushFont(imFont);

      int flags = ImGuiDockNodeFlags.None;
      flags += ImGuiDockNodeFlags.PassthruCentralNode;
//      flags += ImGuiDockNodeFlags.AutoHideTabBar;
      dockspaceId = ImGui.dockSpaceOverViewport(ImGui.getMainViewport(), flags);

      for (ImGuiDockspacePanel dockspacePanel : dockPanelSet)
      {
         dockspacePanel.renderPanel();
      }
   }

   public void renderMenuDockPanelItems()
   {
      ImGui.text("New dock panel:");
      ImGui.sameLine();
      ImGui.pushItemWidth(90.0f);
      ImGui.inputText("###newDockPanelName", newDockPanelName, ImGuiInputTextFlags.CallbackResize);
      ImGui.popItemWidth();
      ImGui.sameLine();
      if (ImGui.button("Create###createNewDockPanelButton"))
      {
         dockPanelSet.add(new ImGuiDockspacePanel(newDockPanelName.get()));
      }

      ImGuiDockspacePanel dockspacePanelToRemove = null;
      for (ImGuiDockspacePanel dockspacePanel : dockPanelSet)
      {
         dockspacePanel.renderMenuItem();
         ImGui.sameLine();
         if (ImGui.button("X###X" + dockspacePanel.getName()))
         {
            dockspacePanelToRemove = dockspacePanel;
         }
      }
      if (dockspacePanelToRemove != null)
      {
         dockPanelSet.remove(dockspacePanelToRemove);
      }
   }

   public void loadSettings(boolean tryUserFirst)
   {
      boolean loaded = tryUserFirst && loadUserSettings();
      if (!loaded)
      {
         if (tryUserFirst)
            LogTools.info("{} not found", imGuiSettingsFile.getExternalFile().toString());
         if (!loadDefaultSettings())
            LogTools.warn("No saved settings found");
      }
   }

   public boolean loadUserSettings()
   {
      boolean success = false;
      if (Files.exists(imGuiSettingsFile.getExternalFile()))
      {
         LogTools.info("Loading ImGui settings from {}", imGuiSettingsFile.getExternalFile().toString());
         ImGui.loadIniSettingsFromDisk(imGuiSettingsFile.getExternalFile().toString());
         success = true;
      }
      return success;
   }

   public boolean loadDefaultSettings()
   {
      boolean success = false;
      if (Files.exists(imGuiSettingsFile.getWorkspaceFile())) // see if there are defaults
      {
         LogTools.info("Loading default ImGui settings from {}", imGuiSettingsFile.getWorkspaceFile().toString());
         ImGui.loadIniSettingsFromDisk(imGuiSettingsFile.getWorkspaceFile().toString());
         success = true;
      }
      return success;
   }

   public void saveSettings(boolean saveDefault)
   {
      Path saveFile = saveDefault ? imGuiSettingsFile.getWorkspaceFile() : imGuiSettingsFile.getExternalFile();
      String settingsPathString = saveFile.toString();
      LogTools.info("Saving ImGui settings to {}", settingsPathString);
      FileTools.ensureDirectoryExists(saveFile.getParent(), DefaultExceptionHandler.PRINT_STACKTRACE);
      ImGui.saveIniSettingsToDisk(settingsPathString);
   }

   public void afterWindowManagement()
   {
      ImGui.popFont();

      ImGui.render();
      imGuiGl3.renderDrawData(ImGui.getDrawData());

      if (imgui.ImGui.getIO().hasConfigFlags(ImGuiConfigFlags.ViewportsEnable)) {
         final long backupWindowPtr = glfwGetCurrentContext();
         imgui.ImGui.updatePlatformWindows();
         imgui.ImGui.renderPlatformWindowsDefault();
         glfwMakeContextCurrent(backupWindowPtr);
      }

      glfwSwapBuffers(windowHandle);
      glfwPollEvents();
   }

   public void dispose()
   {
      imGuiGl3.dispose();
      imGuiGlfw.dispose();

      ImGui.destroyContext();
   }

   public int getCentralDockspaceId()
   {
      return dockspaceId;
   }

   public ImGuiImplGl3 getImGuiGl3()
   {
      return imGuiGl3;
   }
}
