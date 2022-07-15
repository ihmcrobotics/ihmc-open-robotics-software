package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImFont;
import imgui.ImGuiIO;
import imgui.ImGuiStyle;
import imgui.flag.*;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.ImGui;
import imgui.type.ImString;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.system.Callback;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.ImGuiConfigurationLocation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Path;
import java.util.Comparator;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeSet;
import java.util.function.Consumer;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.KHRDebug.GL_DEBUG_SEVERITY_HIGH;

public class GDXImGuiWindowAndDockSystem
{
   public static final String IMGUI_SETTINGS_INI_FILE_NAME = "ImGuiSettings.ini";
   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
   private String glslVersion; // TODO: ?
   private long windowHandle;
   private int fontSizeLevel = 1;
   private ImFont imFont;
   private int dockspaceId;
   private final ImString newDockPanelName = new ImString("", 100);
   private final TreeSet<ImGuiDockspacePanel> dockPanelSet = new TreeSet<>(Comparator.comparing(ImGuiDockspacePanel::getName));
   private final ImGuiPanelManager panelManager;
   private HybridFile imGuiSettingsFile;
   private HybridFile panelsFile;
   private boolean isFirstRenderCall = true;
   private Callback debugMessageCallback;

   public GDXImGuiWindowAndDockSystem()
   {
      panelManager = new ImGuiPanelManager();
   }

   public void setDirectory(HybridDirectory configurationDirectory)
   {
      imGuiSettingsFile = new HybridFile(configurationDirectory, IMGUI_SETTINGS_INI_FILE_NAME);
      panelsFile = new HybridFile(configurationDirectory, "ImGuiPanels.json");
   }

   public void create(long windowHandle)
   {
      this.windowHandle = windowHandle;

      GLFWErrorCallback.createPrint(System.err).set();

      if (!glfwInit())
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);

      // TODO: Something needed here for Mac support?
      // glfwDefaultWindowHints();
      // if (SystemUtils.IS_OS_MAC) {
      //    glslVersion = "#version 150";
      //    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
      //    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
      //    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
      //    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL41.GL_TRUE);            // Required on Mac
      // } else {
      //    glslVersion = "#version 130";
      //    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
      //    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
      // }
      // GL.createCapabilities();

      ImGui.createContext();

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         debugMessageCallback = GDXTools.setupDebugMessageCallback(GL_DEBUG_SEVERITY_HIGH);

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
      imFont = ImGuiTools.setupFonts(io, fontSizeLevel);

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

   public void beforeWindowManagement()
   {
      if (isFirstRenderCall)
      {
         loadUserConfigurationWithDefaultFallback();
      }

      ImGuiTools.glClearDarkGray();
      imGuiGlfw.newFrame();
      ImGui.newFrame();

      ImGui.pushFont(imFont);

      int flags = ImGuiDockNodeFlags.None;
      flags += ImGuiDockNodeFlags.PassthruCentralNode;
//      flags += ImGuiDockNodeFlags.AutoHideTabBar;

      dockspaceId = ImGui.dockSpaceOverViewport(ImGui.getMainViewport(), flags);

      ImGuiDockspacePanel justClosedPanel = null;
      for (ImGuiDockspacePanel dockspacePanel : dockPanelSet)
      {
         dockspacePanel.renderPanel();
         if (dockspacePanel.getWasJustClosed())
         {
            justClosedPanel = dockspacePanel;
            LogTools.debug("Closed dockspace panel: {}", justClosedPanel.getName());
         }
      }

      panelManager.renderPanels(justClosedPanel);
   }

   public void renderMenuDockPanelItems()
   {
      ImGui.text("New dock panel:");
      ImGui.sameLine();
      ImGui.pushItemWidth(90.0f);
      ImGui.inputText("###newDockPanelName", newDockPanelName, ImGuiInputTextFlags.CallbackResize);
      ImGui.popItemWidth();
      ImGui.sameLine();
      if (ImGui.button("Create###createNewDockPanelButton") && !newDockPanelName.get().isEmpty())
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

      ImGui.separator();
      panelManager.renderPanelMenu();
   }

   private void loadUserConfigurationWithDefaultFallback()
   {
      boolean loaded = loadConfiguration(ImGuiConfigurationLocation.USER_HOME);
      if (!loaded)
      {
         LogTools.info("{} not found", imGuiSettingsFile.getExternalFile().toString());
         if (!loadConfiguration(ImGuiConfigurationLocation.VERSION_CONTROL))
            LogTools.warn("No saved settings found");
      }
   }

   public boolean loadConfiguration(ImGuiConfigurationLocation configurationLocation)
   {
      boolean success = false;
      imGuiSettingsFile.setMode(configurationLocation.toHybridResourceMode());
      if (imGuiSettingsFile.isInputStreamAvailable())
      {
         LogTools.info("Loading ImGui settings from {}", imGuiSettingsFile.getLocationOfResourceForReading());
         String iniContentsAsString = ResourceTools.readResourceToString(imGuiSettingsFile.getClasspathResourceAsStream());
         ImGui.loadIniSettingsFromMemory(iniContentsAsString);
         success = true;

         panelsFile.setMode(configurationLocation.toHybridResourceMode());
         LogTools.info("Loading ImGui panels settings from {}", panelsFile.getLocationOfResourceForReading());
         JSONFileTools.load(panelsFile.getInputStream(), jsonNode ->
         {
            JsonNode dockspacePanelsNode = jsonNode.get("dockspacePanels");
            if (dockspacePanelsNode != null)
            {
               ImGuiDockspacePanel[] priorDockpanelSet = dockPanelSet.toArray(new ImGuiDockspacePanel[0]);
               dockPanelSet.clear();
               for (Iterator<Map.Entry<String, JsonNode>> it = dockspacePanelsNode.fields(); it.hasNext(); )
               {
                  Map.Entry<String, JsonNode> dockspacePanelEntry = it.next();
                  ImGuiDockspacePanel dockspacePanel = null;
                  for (ImGuiDockspacePanel otherDockspacePanel : priorDockpanelSet)
                  {
                     if (otherDockspacePanel.getName().equals(dockspacePanelEntry.getKey()))
                     {
                        dockspacePanel = otherDockspacePanel;
                     }
                  }
                  if (dockspacePanel == null)
                  {
                     dockspacePanel = new ImGuiDockspacePanel(dockspacePanelEntry.getKey());
                  }
                  dockPanelSet.add(dockspacePanel);
                  dockspacePanel.getIsShowing().set(dockspacePanelEntry.getValue().asBoolean());
               }
            }
            panelManager.loadConfiguration(jsonNode);
         });
      }
      return success;
   }

   public void saveConfiguration(ImGuiConfigurationLocation saveConfigurationLocation)
   {
      imGuiSettingsFile.setMode(saveConfigurationLocation.toHybridResourceMode());
      Path saveFile = imGuiSettingsFile.getFileForWriting();
      String settingsPathString = saveFile.toString();
      LogTools.info("Saving ImGui settings to {}", settingsPathString);
      FileTools.ensureDirectoryExists(saveFile.getParent(), DefaultExceptionHandler.PRINT_STACKTRACE);
      ImGui.saveIniSettingsToDisk(settingsPathString);

      Consumer<ObjectNode> rootConsumer = root ->
      {
         ObjectNode anchorJSON = root.putObject("dockspacePanels");
         for (ImGuiDockspacePanel dockspacePanel : dockPanelSet)
         {
            anchorJSON.put(dockspacePanel.getName(), dockspacePanel.getIsShowing().get());
         }

         panelManager.saveConfiguration(root);
      };

      panelsFile.setMode(saveConfigurationLocation.toHybridResourceMode());
      LogTools.info("Saving ImGui panel settings to {}", panelsFile.getFileForWriting().toString());
      JSONFileTools.save(panelsFile.getFileForWriting(), rootConsumer);
   }

   public void afterWindowManagement()
   {
      if (isFirstRenderCall)
      {
         JSONFileTools.loadUserWithClasspathDefaultFallback(panelsFile, jsonNode ->
         {
            JsonNode dockspacePanelsNode = jsonNode.get("dockspacePanels");
            if (dockspacePanelsNode != null)
            {
               for (Iterator<Map.Entry<String, JsonNode>> it = dockspacePanelsNode.fields(); it.hasNext(); )
               {
                  Map.Entry<String, JsonNode> dockspacePanelEntry = it.next();
                  ImGuiDockspacePanel dockspacePanel = new ImGuiDockspacePanel(dockspacePanelEntry.getKey());
                  dockspacePanel.getIsShowing().set(dockspacePanelEntry.getValue().asBoolean());
                  dockPanelSet.add(dockspacePanel);
               }
            }
            panelManager.loadConfiguration(jsonNode);
         });
      }

      ImGui.popFont();

      ImGui.render();
      imGuiGl3.renderDrawData(ImGui.getDrawData());

      if (ImGui.getIO().hasConfigFlags(ImGuiConfigFlags.ViewportsEnable))
      {
         final long backupWindowPtr = glfwGetCurrentContext();
         ImGui.updatePlatformWindows();
         ImGui.renderPlatformWindowsDefault();
         glfwMakeContextCurrent(backupWindowPtr);
      }

      isFirstRenderCall = false;
   }

   public void dispose()
   {
      imGuiGl3.dispose();
      imGuiGlfw.dispose();

      ImGui.destroyContext();
      if (debugMessageCallback != null)
         debugMessageCallback.free();
   }

   public ImGuiImplGl3 getImGuiGl3()
   {
      return imGuiGl3;
   }

   public ImGuiPanelManager getPanelManager()
   {
      return panelManager;
   }

   public void setFontSizeLevel(int fontSizeLevel)
   {
      this.fontSizeLevel = fontSizeLevel;
   }

   public ImFont getImFont()
   {
      return imFont;
   }
}
