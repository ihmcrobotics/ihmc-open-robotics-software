package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor.PathType;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Can be embedded in applications for rendering a filtered directory as ImGui radio buttons for easy
 * access to known file types in known locations.
 */
public class ImGuiDirectory
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImString directory = new ImString(ImGuiTools.MAX_STRING_SIZE_FOR_PATH);
   private final Comparator<Path> naturalOrderComparator = Comparator.comparing(path -> path.getFileName().toString());
   private final SortedSet<Path> sortedPaths = new TreeSet<>(naturalOrderComparator.reversed());
   private boolean indexedDirectoryOnce = false;

   private final Function<String, Boolean> radioButtonSelectionCriteria;
   public record PathEntry(Path path, PathType type) { }
   private final Function<PathEntry, Boolean> pathFilter;
   private final Consumer<String> pathSelectedAction;

   public ImGuiDirectory(String initialDirectory,
                         Function<String, Boolean> radioButtonSelectionCriteria,
                         Function<PathEntry, Boolean> pathFilter,
                         Consumer<String> pathSelectedAction)
   {
      this.pathFilter = pathFilter;
      directory.set(initialDirectory);
      this.radioButtonSelectionCriteria = radioButtonSelectionCriteria;
      this.pathSelectedAction = pathSelectedAction;
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.inputText(labels.get("Directory"), directory);

      boolean reindexClicked = ImGui.button(labels.get("Reindex directory"));
      if (!indexedDirectoryOnce || reindexClicked)
      {
         indexedDirectoryOnce = true;
         reindexDirectory();
      }
      ImGui.sameLine();
      ImGui.text("Available paths:");

      ImGui.beginChild(labels.get("Scroll area"), ImGui.getColumnWidth(), ImGuiTools.REASONABLE_HEIGHT_FOR_A_SCROLL_AREA);
      for (Path sortedPath : sortedPaths)
      {
         String pathName = sortedPath.getFileName().toString();
         if (ImGui.radioButton(labels.get(pathName), radioButtonSelectionCriteria.apply(pathName)))
         {
            pathSelectedAction.accept(pathName);
         }
      }
      ImGui.endChild();
   }

   public void reindexDirectory()
   {
      sortedPaths.clear();
      PathTools.walkFlat(Paths.get(directory.get()), (path, type) ->
      {
         if (pathFilter.apply(new PathEntry(path, type)))
            sortedPaths.add(path);
         return FileVisitResult.CONTINUE;
      });
   }

   public String getDirectoryName()
   {
      return directory.get();
   }
}
