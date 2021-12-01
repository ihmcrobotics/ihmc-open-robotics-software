package us.ihmc.gdx.ui;

import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;

import java.nio.file.FileVisitResult;
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
   private final Consumer<Boolean> load;
   private final Consumer<Boolean> save;
   private HybridDirectory perspectiveDirectory;
   private boolean needToReindexPerspectives = true;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString userHomePerspectiveNameToSave = new ImString("", 100);
   private final ImString versionControlPerspectiveNameToSave = new ImString("", 100);
   private final TreeSet<String> userHomePerspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private final TreeSet<String> versionControlPerspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private String currentPerspectiveName = "Main";

   public GDXImGuiPerspectiveManager(Class<?> classForLoading,
                                     String directoryNameToAssumePresent,
                                     String subsequentPathToResourceFolder,
                                     String configurationExtraPath,
                                     HybridDirectory configurationBaseDirectory,
                                     Consumer<HybridDirectory> perspectiveDirectoryUpdated,
                                     Consumer<Boolean> load,
                                     Consumer<Boolean> save)
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
         reindexPerspectives(versionControlPerspectives, configurationBaseDirectory.getWorkspaceDirectory(), true);
         reindexPerspectives(userHomePerspectives, configurationBaseDirectory.getExternalDirectory(), false);
      }

      if (ImGui.beginMenu(labels.get("Perspective")))
      {
         ImGui.text("Version control:");
         renderPerspectiveManager(versionControlPerspectives, true, versionControlPerspectiveNameToSave);

         ImGui.separator();
         ImGui.text("User home:");
         renderPerspectiveManager(userHomePerspectives, false, userHomePerspectiveNameToSave);

         ImGui.separator();
         if (ImGui.button(labels.get("Reindex directories")))
         {
            needToReindexPerspectives = true;
         }
         ImGui.endMenu();
      }
   }

   private void reindexPerspectives(TreeSet<String> perspectives, Path directory, boolean addMainEvenIfItsNotThere)
   {
      perspectives.clear();
      if (addMainEvenIfItsNotThere)
         perspectives.add("Main");
      PathTools.walkFlat(directory, new BasicPathVisitor()
      {
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            if (pathType == PathType.DIRECTORY)
            {
               String directoryName = path.getFileName().toString();
               String matchString = "Perspective";
               if (directoryName.endsWith(matchString))
               {
                  String perspectiveName = directoryName.substring(0, directoryName.lastIndexOf(matchString));
                  LogTools.info("Found perspective {}", perspectiveName);
                  perspectives.add(perspectiveName);
               }
            }
            return FileVisitResult.CONTINUE;
         }
      });
   }

   private void renderPerspectiveManager(TreeSet<String> perspectives, boolean perspectiveDefaultMode, ImString perspectiveNameToSave)
   {
      for (String perspective : perspectives)
      {
         if (ImGui.radioButton(labels.get(perspective, Boolean.toString(perspectiveDefaultMode)), currentPerspectiveName.equals(perspective)))
         {
            currentPerspectiveName = perspective;
            applyPerspectiveDirectory();
            load.accept(perspectiveDefaultMode);
         }
         if (currentPerspectiveName.equals(perspective))
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Save", Boolean.toString(perspectiveDefaultMode), 0)))
            {
               save.accept(perspectiveDefaultMode);
            }
         }
      }

      ImGui.text("Save as:");
      ImGui.sameLine();
      ImGui.inputText(labels.getHidden("NewSaveName" + perspectiveDefaultMode), perspectiveNameToSave, ImGuiInputTextFlags.CallbackResize);
      String perpectiveNameToCreateString = perspectiveNameToSave.get();
      if (!perpectiveNameToCreateString.isEmpty())
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Save", Boolean.toString(perspectiveDefaultMode), 1)))
         {
            String sanitizedName = perpectiveNameToCreateString.replaceAll(" ", "");
            perspectives.add(sanitizedName);
            currentPerspectiveName = sanitizedName;
            applyPerspectiveDirectory();
            perspectiveNameToSave.clear();
            save.accept(perspectiveDefaultMode);
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
}
