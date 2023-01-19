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

public class RDXImGuiPerspectiveManager
{
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;
   private final String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private final ArrayList<Consumer<HybridDirectory>> perspectiveDirectoryUpdatedListeners = new ArrayList<>();
   private final ArrayList<Consumer<ImGuiConfigurationLocation>> loadListeners = new ArrayList<>();
   private final ArrayList<Consumer<ImGuiConfigurationLocation>> saveListeners = new ArrayList<>();
   private HybridDirectory perspectiveDirectory;
   private boolean needToReindexPerspectives = false;
   private boolean firstIndex = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString userHomePerspectiveNameToSave = new ImString("", 100);
   private final ImString versionControlPerspectiveNameToSave = new ImString("", 100);
   private final TreeSet<String> userHomePerspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private final TreeSet<String> versionControlPerspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private String currentPerspectiveName = "Main";
   private ImGuiConfigurationLocation currentConfigurationLocation;

   public RDXImGuiPerspectiveManager(Class<?> classForLoading,
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
      indexPerspectives();
   }

   public void renderImGuiPerspectiveMenu()
   {
      if (needToReindexPerspectives)
      {
         needToReindexPerspectives = false;
         indexPerspectives();
      }

      if (ImGui.beginMenu(labels.get("Perspective")))
      {
         ImGui.text("Version control:");
         renderPerspectiveManager(versionControlPerspectives, ImGuiConfigurationLocation.VERSION_CONTROL, versionControlPerspectiveNameToSave);

         ImGui.separator();
         ImGui.text("User home:");
         renderPerspectiveManager(userHomePerspectives, ImGuiConfigurationLocation.USER_HOME, userHomePerspectiveNameToSave);

         ImGui.separator();
         if (ImGui.button(labels.get("Reindex directories")))
         {
            needToReindexPerspectives = true;
         }
         ImGui.endMenu();
      }
   }

   private void indexPerspectives()
   {
      indexPerspectives(versionControlPerspectives, HybridResourceMode.WORKSPACE, true);
      indexPerspectives(userHomePerspectives, HybridResourceMode.EXTERNAL, false);
      if (firstIndex)
      {
         firstIndex = false;
         if (versionControlPerspectives.contains("Main"))
            currentConfigurationLocation = ImGuiConfigurationLocation.VERSION_CONTROL;
         if (userHomePerspectives.contains("Main"))
            currentConfigurationLocation = ImGuiConfigurationLocation.USER_HOME;
      }
   }

   private void indexPerspectives(TreeSet<String> perspectives, HybridResourceMode resourceMode, boolean addMainEvenIfItsNotThere)
   {
      perspectives.clear();
      if (addMainEvenIfItsNotThere)
         perspectives.add("Main");
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
            perspectives.add("Main");
         }
      }
      for (String directoryName : directoryNames)
      {
         String matchString = "Perspective";
         if (directoryName.endsWith(matchString))
         {
            String perspectiveName = directoryName.substring(0, directoryName.lastIndexOf(matchString));
            LogTools.info("Found perspective {}", perspectiveName);
            perspectives.add(perspectiveName);
         }
      }
   }

   private void renderPerspectiveManager(TreeSet<String> perspectives, ImGuiConfigurationLocation configurationLocation, ImString perspectiveNameToSave)
   {
      boolean enableSaving = configurationLocation == ImGuiConfigurationLocation.USER_HOME
                         || (configurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL && configurationBaseDirectory.isWorkspaceFileAccessAvailable());
      for (String perspective : perspectives)
      {
         if (ImGui.radioButton(labels.get(perspective, configurationLocation.name()),
                               currentPerspectiveName.equals(perspective) && currentConfigurationLocation == configurationLocation))
         {
            currentPerspectiveName = perspective;
            currentConfigurationLocation = configurationLocation;
            applyPerspectiveDirectory();
            for (Consumer<ImGuiConfigurationLocation> loadListener : loadListeners)
            {
               loadListener.accept(configurationLocation);
            }
         }
         if (enableSaving && currentPerspectiveName.equals(perspective))
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
         ImGui.inputText(labels.getHidden("NewSaveName" + configurationLocation.name()), perspectiveNameToSave, ImGuiInputTextFlags.CallbackResize);
         String perpectiveNameToCreateString = perspectiveNameToSave.get();
         if (!perpectiveNameToCreateString.isEmpty())
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Save", configurationLocation.name(), 1)))
            {
               String sanitizedName = perpectiveNameToCreateString.replaceAll(" ", "");
               perspectives.add(sanitizedName);
               currentPerspectiveName = sanitizedName;
               applyPerspectiveDirectory();
               perspectiveNameToSave.clear();
               for (Consumer<ImGuiConfigurationLocation> saveListener : saveListeners)
               {
                  saveListener.accept(configurationLocation);
               }
            }
         }
      }
   }

   public void applyPerspectiveDirectory()
   {
      perspectiveDirectory = new HybridDirectory(dotIHMCDirectory,
                                                 directoryNameToAssumePresent,
                                                 subsequentPathToResourceFolder,
                                                 classForLoading,
                                                 configurationExtraPath + (currentPerspectiveName.equals("Main") ? "" : "/" + currentPerspectiveName
                                                                                                                        + "Perspective"));
      for (Consumer<HybridDirectory> perspectiveDirectoryUpdatedListener : perspectiveDirectoryUpdatedListeners)
      {
         perspectiveDirectoryUpdatedListener.accept(perspectiveDirectory);
      }
   }

   /**
    * This should be called during the update() phase.
    * It might have undesired behavior if called while in the rendering ImGui widgets phase.
    */
   public void reloadPerspective()
   {
      applyPerspectiveDirectory();
      Path directory = currentConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL
            ? perspectiveDirectory.getWorkspaceDirectory() : perspectiveDirectory.getExternalDirectory();
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

   public HybridDirectory getPerspectiveDirectory()
   {
      return perspectiveDirectory;
   }

   public ArrayList<Consumer<HybridDirectory>> getPerspectiveDirectoryUpdatedListeners()
   {
      return perspectiveDirectoryUpdatedListeners;
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
