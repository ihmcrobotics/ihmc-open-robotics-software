package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.commons.nio.BasicPathVisitor.PathType;
import us.ihmc.commons.nio.PathTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Can be embedded in applications for rendering a filtered directory as ImGui radio buttons or drop down menu
 * for easy access to known file types in known locations.
 */
public class ImGuiDirectory
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImString directory = new ImString(ImGuiTools.MAX_STRING_SIZE_FOR_PATH);
   private final Comparator<Path> naturalOrderComparator = Comparator.comparing(path -> path.getFileName().toString());
   private final SortedSet<Path> sortedPaths = new TreeSet<>(naturalOrderComparator.reversed());
   private final SortedSet<String> sortedPathNames = new TreeSet<>();
   private boolean indexedDirectoryOnce = false;

   private Function<String, Boolean> radioButtonSelectionCriteria;
   public record PathEntry(Path path, PathType type) { }
   private final Function<PathEntry, Boolean> pathFilter;
   private final Consumer<String> pathSelectedAction;
   private transient String[] selectablePathNameArray = new String[0];
   private int selectedIndex = 0;

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

   public ImGuiDirectory(String initialDirectory, Function<PathEntry, Boolean> pathFilter, Consumer<String> pathSelectedAction)
   {
      this.pathFilter = pathFilter;
      directory.set(initialDirectory);
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

   public void renderImGuiWidgetsAsDropDownMenu()
   {
      if (!indexedDirectoryOnce)
      {
         indexedDirectoryOnce = true;
         reindexDirectory();
      }

      if (!sortedPathNames.isEmpty())
      {
         selectablePathNameArray = sortedPathNames.toArray(selectablePathNameArray);
         if (ImGui.beginCombo(labels.get("Select item"), selectablePathNameArray[selectedIndex]))
         {
            for (int i = 0; i < selectablePathNameArray.length; i++)
            {
               String pathName = selectablePathNameArray[i];
               if (ImGui.selectable(pathName, selectedIndex == i))
               {
                  selectedIndex = i;
                  pathSelectedAction.accept(pathName);
               }
            }
            ImGui.endCombo();
         }
         else if (selectedIndex == 0)
         {  // when the combo is not empty and initialized to the first entry,
            // we want that to effectively trigger pathSelectedAction with the first entry without having to re-click on it
            String pathName = selectablePathNameArray[0];
            pathSelectedAction.accept(pathName);
         }
      }
   }

   public void reindexDirectory()
   {
      sortedPaths.clear();
      sortedPathNames.clear();
      PathTools.walkFlat(Paths.get(directory.get()), (path, type) ->
      {
         if (pathFilter.apply(new PathEntry(path, type)))
         {
            sortedPaths.add(path);
            sortedPathNames.add(path.getFileName().toString());
         }
         return FileVisitResult.CONTINUE;
      });
   }

   public String getDirectoryName()
   {
      return directory.get();
   }

   public boolean isEmpty()
   {
      if (!indexedDirectoryOnce)
      {
         indexedDirectoryOnce = true;
         reindexDirectory();
      }
      return sortedPaths.isEmpty();
   }
}
