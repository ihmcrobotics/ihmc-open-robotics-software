package us.ihmc.gdx.ui.missionControl;

import imgui.ImGui;
import imgui.extension.texteditor.TextEditor;
import imgui.extension.texteditor.TextEditorLanguageDefinition;
import imgui.type.ImBoolean;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

import java.util.HashMap;

public class ImGuiConsoleArea
{
   private final TextEditor textEditor = new TextEditor();
   private TextEditorLanguageDefinition textEditorLanguageDefinition = new TextEditorLanguageDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean autoScroll = new ImBoolean(true);

   public ImGuiConsoleArea()
   {
      int[] palette = new int[]
            {
                  0xffffffff,	// Default
                  0xffd69c56,	// Keyword
                  0xff00ff00,	// Number
                  0xffffffff,	// String
                  0xffffffff, // Char literal
                  0xffffffff, // Punctuation
                  0xff408080,	// Preprocessor
                  0xffaaaaaa, // Identifier
                  0xff9bc64d, // Known identifier
                  0xffc040a0, // Preproc identifier
                  0xff206020, // Comment (single line)
                  0xffffffff, // Comment (multi line)
                  0xff101010, // Background
                  0xffe0e0e0, // Cursor
                  0x80a06020, // Selection
                  0x800020ff, // ErrorMarker
                  0x40f08000, // Breakpoint
                  0xff707000, // Line number
                  0x40000000, // Current line fill
                  0x40808080, // Current line fill (inactive)
                  0x40a0a0a0, // Current line edge
            };
      textEditor.setPalette(palette);

      HashMap<String, String> identifiers = new HashMap<>();
      textEditorLanguageDefinition = new TextEditorLanguageDefinition();
      textEditorLanguageDefinition.setIdentifiers(identifiers);
      textEditor.setLanguageDefinition(textEditorLanguageDefinition);
      textEditor.setColorizerEnable(true);

      textEditor.setReadOnly(true);
      textEditor.setShowWhitespaces(false);
      textEditor.setHandleMouseInputs(true);
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Auto scroll"), autoScroll);

      if (autoScroll.get())
      {
         int lastLineIndex = textEditor.getTotalLines() - 1;
         int endOfLastLineColumn = textEditor.getTextLines()[lastLineIndex].length();
         textEditor.setCursorPosition(lastLineIndex, endOfLastLineColumn);
      }

      ImGui.pushFont(ImGuiTools.getConsoleFont());
      textEditor.render("Console");
      ImGui.popFont();
   }

   public void acceptNewText(String newText)
   {
      int previousCursorLine = textEditor.getCursorPositionLine();
      int previousCursorColumn = textEditor.getCursorPositionColumn();
      int lastLineIndex = textEditor.getTotalLines() - 1;
      int endOfLastLineColumn = textEditor.getTextLines()[lastLineIndex].length();
      boolean isAutoScroll = previousCursorLine == lastLineIndex && previousCursorColumn == endOfLastLineColumn;
      textEditor.setCursorPosition(lastLineIndex, endOfLastLineColumn);
      textEditor.setReadOnly(false);
      textEditor.insertText(newText);
      textEditor.setReadOnly(true);
      if (!isAutoScroll)
         textEditor.setCursorPosition(previousCursorLine, previousCursorColumn);
   }
}
