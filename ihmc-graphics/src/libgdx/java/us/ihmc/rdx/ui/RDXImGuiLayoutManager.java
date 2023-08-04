package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridResourceDirectory;
import us.ihmc.tools.io.HybridResourceMode;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.Consumer;
import java.util.function.Function;

public class RDXImGuiLayoutManager
{
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private final Class<?> classForLoading;
   private final String configurationExtraPath;
   private final HybridResourceDirectory configurationBaseDirectory;
   private final ArrayList<Consumer<HybridResourceDirectory>> layoutDirectoryUpdatedListeners = new ArrayList<>();
   private final ArrayList<Function<ImGuiConfigurationLocation, Boolean>> loadListeners = new ArrayList<>();
   private final ArrayList<Consumer<ImGuiConfigurationLocation>> saveListeners = new ArrayList<>();
   private HybridResourceDirectory layoutDirectory;
   private boolean needToReindexLayouts = false;
   private boolean firstIndex = true;
   private boolean layoutHasBeenLoadedOnce = false;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString userHomeLayoutNameToSave = new ImString(100);
   private final ImString versionControlLayoutNameToSave = new ImString(100);
   private final TreeSet<String> userHomeLayouts = new TreeSet<>(Comparator.comparing(String::toString));
   private final TreeSet<String> versionControlLayouts = new TreeSet<>(Comparator.comparing(String::toString));
   private String currentLayoutName = "Main";
   private ImGuiConfigurationLocation currentConfigurationLocation;

   public RDXImGuiLayoutManager(Class<?> classForLoading, String configurationExtraPath, HybridResourceDirectory configurationBaseDirectory)
   {
      this.classForLoading = classForLoading;
      this.configurationExtraPath = configurationExtraPath;
      this.configurationBaseDirectory = configurationBaseDirectory;
      indexLayouts();
   }

   public void renderImGuiLayoutMenu()
   {
      if (needToReindexLayouts)
      {
         needToReindexLayouts = false;
         indexLayouts();
      }

      if (ImGui.beginMenu(labels.get("Layout")))
      {
         ImGui.text("Version control:");
         renderLayoutManager(versionControlLayouts, ImGuiConfigurationLocation.VERSION_CONTROL, versionControlLayoutNameToSave);

         ImGui.separator();
         ImGui.text("User home:");
         renderLayoutManager(userHomeLayouts, ImGuiConfigurationLocation.USER_HOME, userHomeLayoutNameToSave);

         ImGui.separator();
         if (ImGui.button(labels.get("Reindex directories")))
         {
            needToReindexLayouts = true;
         }
         ImGui.endMenu();
      }
   }

   private void indexLayouts()
   {
      indexLayouts(versionControlLayouts, HybridResourceMode.WORKSPACE, true);
      indexLayouts(userHomeLayouts, HybridResourceMode.EXTERNAL, false);
      if (firstIndex)
      {
         firstIndex = false;
         if (versionControlLayouts.contains("Main"))
            currentConfigurationLocation = ImGuiConfigurationLocation.VERSION_CONTROL;
         if (userHomeLayouts.contains("Main")) // This being second makes user home the default if it exists
            currentConfigurationLocation = ImGuiConfigurationLocation.USER_HOME;
      }
   }

   private void indexLayouts(TreeSet<String> layouts, HybridResourceMode resourceMode, boolean addMainEvenIfItsNotThere)
   {
      layouts.clear();
      if (addMainEvenIfItsNotThere)
         layouts.add("Main");
      TreeSet<String> fileNames = new TreeSet<>();
      TreeSet<String> directoryNames = new TreeSet<>();
      if (resourceMode == HybridResourceMode.WORKSPACE)
      {
         configurationBaseDirectory.walkResourcesFlat((path, pathType) ->
         {
            if (pathType == BasicPathVisitor.PathType.DIRECTORY)
               directoryNames.add(path);
            else
               fileNames.add(path);
         });
      }
      else
      {
         PathTools.walkFlat(configurationBaseDirectory.getExternalDirectory(), new BasicPathVisitor()
         {
            @Override
            public FileVisitResult visitPath(Path path, PathType pathType)
            {
               if (pathType == PathType.FILE)
               {
                  String fileName = path.getFileName().toString();
                  fileNames.add(fileName);
               }
               if (pathType == PathType.DIRECTORY)
               {
                  String directoryName = path.getFileName().toString();
                  directoryNames.add(directoryName);
               }
               return FileVisitResult.CONTINUE;
            }
         });
      }
      for (String fileName : fileNames)
      {
         if (fileName.equals(RDXImGuiWindowAndDockSystem.IMGUI_SETTINGS_INI_FILE_NAME))
         {
            layouts.add("Main");
         }
      }
      for (String directoryName : directoryNames)
      {
         String matchString = "Layout";
         if (directoryName.endsWith(matchString))
         {
            String layoutName = directoryName.substring(0, directoryName.lastIndexOf(matchString));
            LogTools.info("Found layout {}", layoutName);
            layouts.add(layoutName);
         }
      }
   }

   private void renderLayoutManager(TreeSet<String> layouts, ImGuiConfigurationLocation configurationLocation, ImString layoutNameToSave)
   {
      boolean enableSaving = configurationLocation.isUserHome() ||
                             (configurationLocation.isVersionControl() && configurationBaseDirectory.isWorkspaceFileAccessAvailable());
      for (String layout : layouts)
      {
         if (ImGui.radioButton(labels.get(layout, configurationLocation.name()),
                               currentLayoutName.equals(layout) && currentConfigurationLocation == configurationLocation))
         {
            currentLayoutName = layout;
            currentConfigurationLocation = configurationLocation;
            applyLayoutDirectory();
            loadConfiguration(configurationLocation);
         }
         if (enableSaving && currentLayoutName.equals(layout))
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Save", configurationLocation.name(), 0)))
            {
               for (Consumer<ImGuiConfigurationLocation> saveListener : saveListeners)
               {
                  saveListener.accept(configurationLocation);
               }
            }
         }
      }

      if (enableSaving)
      {
         ImGui.text("Save as:");
         ImGui.sameLine();
         // A little fanciness here to make the menu a constant width whether or not
         // rendering the save button
         String layoutNameToCreateString = layoutNameToSave.get();
         boolean renderSaveButton = layoutNameToCreateString.isEmpty();
         ImGui.setNextItemWidth(renderSaveButton ? 190.0f : 150.0f); // Width of Save button is 40
         boolean saveRequested = ImGuiTools.inputText(labels.getHidden("NewSaveName" + configurationLocation.name()), layoutNameToSave);
         ImGuiTools.previousWidgetTooltip("A PascalCased name is recommended.\nPress Enter to save.");
         layoutNameToCreateString = layoutNameToSave.get();
         if (!renderSaveButton)
         {
            ImGui.sameLine();
            saveRequested |= ImGui.button(labels.get("Save", configurationLocation.name(), 1));
            if (saveRequested)
            {
               String sanitizedName = layoutNameToCreateString.replaceAll(" ", "");
               layouts.add(sanitizedName);
               currentLayoutName = sanitizedName;
               applyLayoutDirectory();
               layoutNameToSave.clear();
               for (Consumer<ImGuiConfigurationLocation> saveListener : saveListeners)
               {
                  saveListener.accept(configurationLocation);
               }
            }
         }
      }
   }

   private void loadConfiguration(ImGuiConfigurationLocation configurationLocation)
   {
      boolean success = true;
      for (Function<ImGuiConfigurationLocation, Boolean> loadListener : loadListeners)
      {
         success &= loadListener.apply(configurationLocation);
      }
      if (!success && configurationLocation.isVersionControl())
      {
         LogTools.warn("Layout configuration file(s) not found. If you just created this layout, "
                        + "try building in the IDE to copy the resources to a classpath directory.");
      }
   }

   public void applyLayoutDirectory()
   {
      layoutDirectory = new HybridResourceDirectory(dotIHMCDirectory, classForLoading, "/");
      layoutDirectory = layoutDirectory.resolve(configurationExtraPath);
      if (!currentLayoutName.equals("Main"))
         layoutDirectory = layoutDirectory.resolve(currentLayoutName + "Layout");
      for (Consumer<HybridResourceDirectory> layoutDirectoryUpdatedListener : layoutDirectoryUpdatedListeners)
      {
         layoutDirectoryUpdatedListener.accept(layoutDirectory);
      }
   }

   /**
    * Should only be called by before rendering the first ImGui frame.
    */
   public void loadInitialLayout()
   {
      if (!layoutHasBeenLoadedOnce)
      {
         layoutHasBeenLoadedOnce = true;
         LogTools.info(1, "Loading layout.");
         reloadLayoutInternal();
      }
   }

   /**
    * This should be called during the update() phase.
    * It might have undesired behavior if called while in the rendering ImGui widgets phase.
    */
   public void reloadLayout()
   {
      if (layoutHasBeenLoadedOnce)
      {
         LogTools.info(1, "Reloading layout.");
         reloadLayoutInternal();
      }
   }

   private void reloadLayoutInternal()
   {
      applyLayoutDirectory();
      loadConfiguration(currentConfigurationLocation);
   }

   public ImGuiConfigurationLocation getCurrentConfigurationLocation()
   {
      return currentConfigurationLocation;
   }

   public HybridResourceDirectory getLayoutDirectory()
   {
      return layoutDirectory;
   }

   public ArrayList<Consumer<HybridResourceDirectory>> getLayoutDirectoryUpdatedListeners()
   {
      return layoutDirectoryUpdatedListeners;
   }

   public ArrayList<Consumer<ImGuiConfigurationLocation>> getSaveListeners()
   {
      return saveListeners;
   }

   public ArrayList<Function<ImGuiConfigurationLocation, Boolean>> getLoadListeners()
   {
      return loadListeners;
   }
}
