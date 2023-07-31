package us.ihmc.rdx;

import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiKey;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImString;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.tools.string.StringTools;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

/**
 * Allows you to register key bindings which get rendered in a table menu
 */
public class RDXKeyBindings
{
   private static final int WINDOW_WIDTH = 600;
   private static final int WINDOW_HEIGHT = 400;

   private final Map<String, KeyBindingsSection> sections = new HashMap<>();

   // Search filter
   private final ImString filter = new ImString();
   private boolean filterInputActive = false;
   private boolean forceActive = false;

   private static class KeyBindingsSection
   {
      private final String description;
      // Function to key map
      private final TreeMap<String, String> registry = new TreeMap<>();

      private KeyBindingsSection(String description)
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
            {
               boolean match = false;

               if (function.toLowerCase().contains(filter.get().toLowerCase()))
               {
                  match = true;
               }

               if (description.toLowerCase().contains(filter.get().toLowerCase()))
               {
                  match = true;
               }

               if (!match)
                  continue;
            }

            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text(function);
            ImGui.tableSetColumnIndex(1);
            ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
            ImGui.button(key, 85, 0);
            ImGui.popItemFlag();
         }
      }
   }

   /**
    * Register a new key bind
    *
    * @param function some description of what the key bind does
    * @param key      the key (examples: "A", "Ctrl + Z", "Space")
    * @return true if registered a new key binding, false if it already exists
    */
   public boolean register(String function, String key)
   {
      String callingClassName = getCallingClassName();

      final KeyBindingsSection section;

      if (sections.containsKey(callingClassName))
      {
         section = sections.get(callingClassName);
      }
      else
      {
         String description = StringTools.pascalCaseToSentenceCase(callingClassName.replace("RDX", ""));
         section = new KeyBindingsSection(description);
         sections.put(callingClassName, section);
      }

      if (!section.registry.containsKey(function))
      {
         section.registry.put(function, key);
         return true;
      }

      return false;
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

      if (tabKeyIsDown || filterInputActive || forceActive)
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

            if (forceActive)
            {
               ImGui.setKeyboardFocusHere(0);
            }

            if (ImGui.inputText("Search", filter))
            {
               filterInputActive = true;
            }

            filterInputActive = ImGui.isItemFocused() || forceActive;

            if (!filterInputActive)
               filter.set("");

            ImGui.endTable();
         }

         for (Map.Entry<String, KeyBindingsSection> entry : sections.entrySet())
         {
            KeyBindingsSection section = entry.getValue();

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

         forceActive = false;
      }
   }

   public void showKeybindings()
   {
      forceActive = true;
      filterInputActive = true;
   }

   private static String getCallingClassName()
   {
      StackTraceElement callerStackElement = new Throwable().getStackTrace()[2];
      return callerStackElement.getClassName().substring(callerStackElement.getClassName().lastIndexOf(".") + 1);
   }
}
