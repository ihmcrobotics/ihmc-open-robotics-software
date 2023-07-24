package us.ihmc.rdx;

import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiKey;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import imgui.type.ImString;
import us.ihmc.rdx.imgui.ImGuiTools;

import javax.annotation.Nullable;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

public class RDXKeyBindings
{
   private static final int WINDOW_WIDTH = 600;
   private static final int WINDOW_HEIGHT = 400;

   @Nullable
   private KeyBindingsSection currentSection;
   private final Set<KeyBindingsSection> sections = new HashSet<>();

   // Search filter
   private final ImString filter = new ImString();
   private boolean filterInputActive = false;

   private static class KeyBindingsSection
   {
      private final String description;
      // Function to key map
      private final TreeMap<String, String> registry = new TreeMap<>();

      public KeyBindingsSection(String description)
      {
         this.description = description;
      }

      public void renderSection(ImString filter)
      {
         for (Map.Entry<String, String> entry : registry.entrySet())
         {
            String function = entry.getKey();
            String key = entry.getValue();

            if (!filter.isEmpty())
               if (!function.toLowerCase().contains(filter.get()))
                  continue;

            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text(function);
            ImGui.tableSetColumnIndex(1);
            ImGui.button(key, 80, 0);
         }
      }
   }

   public boolean nextSection(String sectionName)
   {
      if (currentSection != null)
      {
         for (KeyBindingsSection section : sections)
         {
            if (section.description.equals(sectionName))
               return false;
         }

         sections.add(currentSection);
      }
      currentSection = new KeyBindingsSection(sectionName);

      return true;
   }

   public void register(String function, String key) throws IllegalArgumentException
   {
      if (currentSection == null)
      {
         throw new IllegalStateException("You must call RDXKeyBindings#nextSection() first before registering a key bind.");
      }
      currentSection.registry.put(function, key);
   }

   public void renderKeyBindingsTable()
   {
      // Escape should close the keybindings menu if it's staying active because of the filter input text box being active
      if (filterInputActive)
      {
         if (ImGui.isKeyPressed(ImGui.getKeyIndex(ImGuiKey.Escape)))
         {
            filterInputActive = false;
         }
      }

      boolean tabKeyIsDown = ImGui.isKeyDown(ImGui.getKeyIndex(ImGuiKey.Tab));

      if (tabKeyIsDown || filterInputActive)
      {
         ImGui.setNextWindowViewport(ImGui.getMainViewport().getID());
         float x = ImGui.getMainViewport().getCenterX() - (WINDOW_WIDTH * 0.5f);
         float y = ImGui.getMainViewport().getCenterY() - (WINDOW_HEIGHT * 0.5f);
         ImGui.setNextWindowPos(x, y, ImGuiCond.Always);
         ImGui.begin("##keyBindingsWindow", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize);
         ImGui.beginChild("##keyBindingsWindowChild", WINDOW_WIDTH, WINDOW_HEIGHT);

         if (ImGui.beginTable("##keyBindingsSearch", 2))
         {
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.pushFont(ImGuiTools.getMediumFont());
            ImGui.text("Key Bindings");
            ImGui.popFont();
            ImGui.tableSetColumnIndex(1);

            if (ImGui.inputText("Search", filter))
            {
               filterInputActive = true;
            }

            filterInputActive = ImGui.isItemFocused();

            if (!filterInputActive)
               filter.set("");

            ImGui.endTable();
         }

         for (KeyBindingsSection section : sections)
         {
            if (ImGui.beginTable("##keyBindingsTable_" + section.description, 2))
            {
               ImGui.tableSetupColumn(section.description);
               ImGui.tableSetupColumn("Key");
               ImGui.tableHeadersRow();
               section.renderSection(filter);
               ImGui.endTable();
            }
         }

         ImGui.endChild();
         ImGui.end();
      }
   }
}
