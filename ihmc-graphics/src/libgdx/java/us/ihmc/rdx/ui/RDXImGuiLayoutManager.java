package us.ihmc.rdx.ui;

import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridResourceMode;

import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.Consumer;

public class RDXImGuiLayoutManager
{
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;
   private final String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private final ArrayList<Consumer<HybridDirectory>> layoutDirectoryUpdatedListeners = new ArrayList<>();
   private final ArrayList<Consumer<ImGuiConfigurationLocation>> loadListeners = new ArrayList<>();
   private final ArrayList<Consumer<ImGuiConfigurationLocation>> saveListeners = new ArrayList<>();
   private HybridDirectory layoutDirectory;
   private boolean needToReindexLayouts = false;
   private boolean firstIndex = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString userHomeLayoutNameToSave = new ImString("", 100);
   private final ImString versionControlLayoutNameToSave = new ImString("", 100);
   private final TreeSet<String> userHomeLayouts = new TreeSet<>(Comparator.comparing(String::toString));
   private final TreeSet<String> versionControlLayouts = new TreeSet<>(Comparator.comparing(String::toString));
   private String currentLayoutName = "Main";
   private ImGuiConfigurationLocation currentConfigurationLocation;

   public RDXImGuiLayoutManager(Class<?> classForLoading,
                                String directoryNameToAssumePresent,
                                String subsequentPathToResourceFolder,
                                String configurationExtraPath,
                                HybridDirectory configurationBaseDirectory)
   {
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
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
         if (userHomeLayouts.contains("Main"))
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
      boolean enableSaving = configurationLocation == ImGuiConfigurationLocation.USER_HOME
                         || (configurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL && configurationBaseDirectory.isWorkspaceFileAccessAvailable());
      for (String layout : layouts)
      {
         if (ImGui.radioButton(labels.get(layout, configurationLocation.name()),
                               currentLayoutName.equals(layout) && currentConfigurationLocation == configurationLocation))
         {
            currentLayoutName = layout;
            currentConfigurationLocation = configurationLocation;
            applyLayoutDirectory();
            for (Consumer<ImGuiConfigurationLocation> loadListener : loadListeners)
            {
               loadListener.accept(configurationLocation);
            }
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
         ImGui.inputText(labels.getHidden("NewSaveName" + configurationLocation.name()), layoutNameToSave, ImGuiInputTextFlags.CallbackResize);
         String layoutNameToCreateString = layoutNameToSave.get();
         if (!layoutNameToCreateString.isEmpty())
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Save", configurationLocation.name(), 1)))
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

   public void applyLayoutDirectory()
   {
      layoutDirectory = new HybridDirectory(dotIHMCDirectory,
                                            directoryNameToAssumePresent,
                                            subsequentPathToResourceFolder,
                                            classForLoading,
                                                 configurationExtraPath + (currentLayoutName.equals("Main") ? "" : "/" + currentLayoutName
                                                                                                                   + "Layout"));
      for (Consumer<HybridDirectory> layoutDirectoryUpdatedListener : layoutDirectoryUpdatedListeners)
      {
         layoutDirectoryUpdatedListener.accept(layoutDirectory);
      }
   }

   /**
    * This should be called during the update() phase.
    * It might have undesired behavior if called while in the rendering ImGui widgets phase.
    */
   public void reloadLayout()
   {
      applyLayoutDirectory();
      Path directory = currentConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL
            ? layoutDirectory.getWorkspaceDirectory() : layoutDirectory.getExternalDirectory();
      if (Files.exists(directory))
      {
         for (Consumer<ImGuiConfigurationLocation> loadListener : loadListeners)
         {
            loadListener.accept(currentConfigurationLocation);
         }
      }
   }

   public ImGuiConfigurationLocation getCurrentConfigurationLocation()
   {
      return currentConfigurationLocation;
   }

   public HybridDirectory getLayoutDirectory()
   {
      return layoutDirectory;
   }

   public ArrayList<Consumer<HybridDirectory>> getLayoutDirectoryUpdatedListeners()
   {
      return layoutDirectoryUpdatedListeners;
   }

   public ArrayList<Consumer<ImGuiConfigurationLocation>> getSaveListeners()
   {
      return saveListeners;
   }

   public ArrayList<Consumer<ImGuiConfigurationLocation>> getLoadListeners()
   {
      return loadListeners;
   }
}
