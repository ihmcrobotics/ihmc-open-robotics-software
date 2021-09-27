package us.ihmc.gdx.ui;

import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
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
   private final ImString perspectiveNameToSave = new ImString("", 100);
   private final ImBoolean perspectiveDefaultMode = new ImBoolean(false);
   private final TreeSet<String> perspectives = new TreeSet<>(Comparator.comparing(String::toString));
   private String currentPerspective = "Main";

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
         Path directory = perspectiveDefaultMode.get() ? configurationBaseDirectory.getWorkspaceDirectory() : configurationBaseDirectory.getExternalDirectory();
         perspectives.clear();
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

      if (ImGui.beginMenu("Perspective"))
      {
         for (String perspective : perspectives)
         {
            if (ImGui.radioButton(perspective, currentPerspective.equals(perspective)))
            {
               currentPerspective = perspective;
               applyPerspectiveDirectory();
               load.accept(perspectiveDefaultMode.get());
            }
            if (currentPerspective.equals(perspective))
            {
               ImGui.sameLine();
               if (ImGui.button("Save"))
               {
                  save.accept(perspectiveDefaultMode.get());
               }
            }
         }

         ImGui.text("Save as:");
         ImGui.sameLine();
         ImGui.inputText("###", perspectiveNameToSave , ImGuiInputTextFlags.CallbackResize);
         String perpectiveNameToCreateString = perspectiveNameToSave.get();
         if (!perpectiveNameToCreateString.isEmpty())
         {
            ImGui.sameLine();
            if (ImGui.button("Save"))
            {
               String sanitizedName = perpectiveNameToCreateString.replaceAll(" ", "");
               perspectives.add(sanitizedName);
               currentPerspective = sanitizedName;
               applyPerspectiveDirectory();
               perspectiveNameToSave.clear();
               save.accept(perspectiveDefaultMode.get());
            }
         }

         ImGui.separator();
         ImGui.text("Save location:");
         if (ImGui.radioButton("User home###PerspectiveUserHomeMode", !perspectiveDefaultMode.get()))
         {
            perspectiveDefaultMode.set(false);
            needToReindexPerspectives = true;
         }
         ImGui.sameLine();
         if (ImGui.radioButton("Version control###PerspectiveDefaultMode", perspectiveDefaultMode.get()))
         {
            perspectiveDefaultMode.set(true);
            needToReindexPerspectives = true;
         }
         ImGui.endMenu();
      }
   }

   private void applyPerspectiveDirectory()
   {
      perspectiveDirectory = new HybridDirectory(dotIHMCDirectory,
                                                 directoryNameToAssumePresent,
                                                 subsequentPathToResourceFolder,
                                                 classForLoading,
                                                 configurationExtraPath + (currentPerspective.equals("Main") ? "" : "/" + currentPerspective + "Perspective"));
      perspectiveDirectoryUpdated.accept(perspectiveDirectory);
   }
}
