package us.ihmc.gdx.ui;

import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.gdx.imgui.GDXImGuiWindowAndDockSystem;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridResourceMode;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.Consumer;

public class GDXImGuiPerspectiveManager
{
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;
   private final String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private final Consumer<HybridDirectory> perspectiveDirectoryUpdated;
   private final Consumer<ImGuiConfigurationLocation> load;
   private final Consumer<ImGuiConfigurationLocation> save;
   private HybridDirectory perspectiveDirectory;
   private boolean needToReindexPerspectives = true;
   private boolean firstReindex = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString userHomePerspectiveNameToSave = new ImString("", 100);
   private final ImString versionControlPerspectiveNameToSave = new ImString("", 100);
   private final TreeSet<String> userHomePerspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private final TreeSet<String> versionControlPerspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private String currentPerspectiveName = "Main";
   private ImGuiConfigurationLocation currentConfigurationLocation;

   public GDXImGuiPerspectiveManager(Class<?> classForLoading,
                                     String directoryNameToAssumePresent,
                                     String subsequentPathToResourceFolder,
                                     String configurationExtraPath,
                                     HybridDirectory configurationBaseDirectory,
                                     Consumer<HybridDirectory> perspectiveDirectoryUpdated,
                                     Consumer<ImGuiConfigurationLocation> load,
                                     Consumer<ImGuiConfigurationLocation> save)
   {
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
      this.configurationExtraPath = configurationExtraPath;
      this.configurationBaseDirectory = configurationBaseDirectory;
      this.perspectiveDirectoryUpdated = perspectiveDirectoryUpdated;
      this.load = load;
      this.save = save;

      applyPerspectiveDirectory();
   }

   public void renderImGuiPerspectiveMenu()
   {
      if (needToReindexPerspectives)
      {
         needToReindexPerspectives = false;
         reindexPerspectives(versionControlPerspectives, HybridResourceMode.WORKSPACE, true);
         reindexPerspectives(userHomePerspectives, HybridResourceMode.EXTERNAL, false);
         if (firstReindex)
         {
            firstReindex = false;
            if (versionControlPerspectives.contains("Main"))
               currentConfigurationLocation = ImGuiConfigurationLocation.VERSION_CONTROL;
            if (userHomePerspectives.contains("Main"))
               currentConfigurationLocation = ImGuiConfigurationLocation.USER_HOME;
         }
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

   private void reindexPerspectives(TreeSet<String> perspectives, HybridResourceMode resourceMode, boolean addMainEvenIfItsNotThere)
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
               fileNames.add(path);;
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
         if (fileName.equals(GDXImGuiWindowAndDockSystem.IMGUI_SETTINGS_INI_FILE_NAME))
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
      for (String perspective : perspectives)
      {
         if (ImGui.radioButton(labels.get(perspective, configurationLocation.name()),
                               currentPerspectiveName.equals(perspective) && currentConfigurationLocation == configurationLocation))
         {
            currentPerspectiveName = perspective;
            currentConfigurationLocation = configurationLocation;
            applyPerspectiveDirectory();
            load.accept(configurationLocation);
         }
         if (currentPerspectiveName.equals(perspective))
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Save", configurationLocation.name(), 0)))
            {
               save.accept(configurationLocation);
            }
         }
      }

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
            save.accept(configurationLocation);
         }
      }
   }

   private void applyPerspectiveDirectory()
   {
      perspectiveDirectory = new HybridDirectory(dotIHMCDirectory,
                                                 directoryNameToAssumePresent,
                                                 subsequentPathToResourceFolder,
                                                 classForLoading,
                                                 configurationExtraPath + (currentPerspectiveName.equals("Main") ? "" : "/" + currentPerspectiveName
                                                                                                                        + "Perspective"));
      perspectiveDirectoryUpdated.accept(perspectiveDirectory);
   }

   public void reloadPerspective()
   {
      applyPerspectiveDirectory();
      Path directory = currentConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL
            ? perspectiveDirectory.getWorkspaceDirectory() : perspectiveDirectory.getExternalDirectory();
      if (Files.exists(directory))
      {
         load.accept(currentConfigurationLocation);
      }
   }
}
