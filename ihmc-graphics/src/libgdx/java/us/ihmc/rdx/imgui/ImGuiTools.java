package us.ihmc.rdx.imgui;

import com.badlogic.gdx.Input;
import com.badlogic.gdx.graphics.Color;
import imgui.*;
import imgui.flag.*;
import imgui.internal.ImGuiContext;
import imgui.type.*;
import org.apache.commons.lang3.SystemUtils;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.tools.string.StringTools;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Clean up to have lots of different font sizes available by number.
 */
public class ImGuiTools
{
   private static final AtomicInteger GLOBAL_WIDGET_INDEX = new AtomicInteger();
   public static float TAB_BAR_HEIGHT = 20.0f;
   public static final int GDX_TO_IMGUI_KEY_CODE_OFFSET = GLFW.GLFW_KEY_A - Input.Keys.A;
   public static final float FLOAT_MIN = -3.40282346638528859811704183484516925e+38F / 2.0f;
   public static final float FLOAT_MAX = 3.40282346638528859811704183484516925e+38F / 2.0f;
   /** This is used so a scroll area can end reasonably for long scrolls so stuff below it can be accessed. */
   public static final float REASONABLE_HEIGHT_FOR_A_SCROLL_AREA = 150.0f;
   public static final int MAX_STRING_SIZE_FOR_PATH = 1024;

   private static ImFont consoleFont;
   private static ImFont smallFont;
   private static ImFont smallBoldFont;
   private static ImFont mediumFont;
   private static ImFont bigFont;
   private static ImFont nodeFont;

   private static boolean userKeysHaveBeenMapped = false;
   private static int spaceKey;
   private static int deleteKey;
   private static int escapeKey;
   private static int enterKey;
   private static int upArrowKey;
   private static int downArrowKey;
   private static int leftArrowKey;
   private static int rightArrowKey;
   private static ImFontAtlas fontAtlas;

   public static int BLACK = Color.BLACK.toIntBits();
   public static int WHITE = Color.WHITE.toIntBits();
   public static int GRAY = Color.GRAY.toIntBits();
   public static int RED = Color.RED.toIntBits();
   public static int GREEN = Color.GREEN.toIntBits();
   public static int DARK_RED = new Color(0.7f, 0.0f, 0.0f, 1.0f).toIntBits();
   public static int DARK_GREEN = new Color(0.0f, 0.7f, 0.0f, 1.0f).toIntBits();
   public static int LIGHT_BLUE = new Color(0.4f, 0.4f, 0.8f, 1.0f).toIntBits();

   private static final ImVec2 calcTextSize = new ImVec2();

   public static long createContext()
   {
      return ImGui.createContext().ptr;
   }

   public static long createContext(ImFontAtlas fontAtlas)
   {
      return ImGui.createContext(fontAtlas).ptr;
   }

   public static long getCurrentContext()
   {
      return ImGui.getCurrentContext().ptr;
   }

   public static void setCurrentContext(long context)
   {
      ImGuiContext contextHolder = ImGui.getCurrentContext();
      contextHolder.ptr = context;
      ImGui.setCurrentContext(contextHolder);
   }

   public static void parsePrimaryWindowSizeFromSettingsINI(String settingsINIAsString, ImGuiSize sizeToPack)
   {
      settingsINIAsString = StringTools.filterOutCRLFLineEndings(settingsINIAsString);
      int indexOfDockingSection = settingsINIAsString.indexOf("[Docking]");
      int indexOfDockspace = settingsINIAsString.indexOf("DockSpace", indexOfDockingSection); // The first DockSpace entry is the primary one
      int indexOfSize = settingsINIAsString.indexOf("Size", indexOfDockspace);
      int indexOfWidth = indexOfSize + 5; // Account for '='
      int indexOfComma = settingsINIAsString.indexOf(",", indexOfWidth);
      int width = Integer.parseInt(settingsINIAsString.substring(indexOfWidth, indexOfComma));

      int indexOfHeight = indexOfComma + 1; // Account for ','
      int indexOfSpace = settingsINIAsString.indexOf(" ", indexOfComma);
      int height = Integer.parseInt(settingsINIAsString.substring(indexOfHeight, indexOfSpace));

      sizeToPack.setWidth(width);
      sizeToPack.setHeight(height);
   }

   public static void parsePrimaryWindowPositionFromSettingsINI(String settingsINIAsString, ImGuiPosition positionToPack)
   {
      settingsINIAsString = StringTools.filterOutCRLFLineEndings(settingsINIAsString);
      int indexOfDockingSection = settingsINIAsString.indexOf("[Docking]");
      int indexOfDockspace = settingsINIAsString.indexOf("DockSpace", indexOfDockingSection); // The first DockSpace entry is the primary one
      int indexOfPosition = settingsINIAsString.indexOf("Pos", indexOfDockspace);
      int indexOfX = indexOfPosition + 4; // Account for '='
      int indexOfComma = settingsINIAsString.indexOf(",", indexOfX);
      int x = Integer.parseInt(settingsINIAsString.substring(indexOfX, indexOfComma));

      int indexOfY = indexOfComma + 1; // Account for ','
      int indexOfSpace = settingsINIAsString.indexOf(" ", indexOfComma);
      int y = Integer.parseInt(settingsINIAsString.substring(indexOfY, indexOfSpace));

      positionToPack.setX(x);
      positionToPack.setY(y);
   }

   public static void initializeColorStyle()
   {
      if (!Boolean.parseBoolean(System.getProperty("imgui.dark")))
         ImGui.styleColorsLight();
   }

   /**
    * Method for getting color ranging from green to red based on value inputted (greater value = more red)
    * Values at which color changes are provided using the colorSwitchValues varargs.
    * e.g.
    *    greenToRedGradiatedColor(0.5, 0.7, 0.9) -> returns green color       (0.5 < 0.7)
    *    greenToRedGradiatedColor(0.8, 0.7, 0.9) -> returns orange-ish color  (0.7 < 0.8 < 0.9)
    *    greenToRedGradiatedColor(1.0, 0.7, 0.9) -> returns red color         (1.0 > 0.9)
    *
    * @param value The value which determines returned color
    * @param colorSwitchValues values at which color changes (if given 3 values, color will switch from green -> yellow -> orange -> red)
    * @return Integer value representing color
    */
   public static int greenToRedGradiatedColor(double value, double... colorSwitchValues)
   {
      float redValue = 0.0f;
      float greenValue = 1.0f;

      for (double switchValue : colorSwitchValues)
      {
         if (value < switchValue)
            break;

         redValue = 1.0f;
         greenValue -= 1.0 / colorSwitchValues.length;
      }

      return new Color(redValue, greenValue, 0.0f, 0.5f).toIntBits();
   }

   public static int nextWidgetIndex()
   {
      return GLOBAL_WIDGET_INDEX.getAndIncrement();
   }

   public static void glClearDarkGray()
   {
      GL41.glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT);
   }

   public static boolean volatileInputInt(String label, ImInt imInt)
   {
      return volatileInputInt(label, imInt, 1);
   }

   public static boolean volatileInputInt(String label, ImInt imInt, int step)
   {
      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags += ImGuiInputTextFlags.EnterReturnsTrue;
      return ImGui.inputInt(label, imInt, step, 100, inputTextFlags);
   }

   public static boolean volatileInputLong(String label, ImLong imLong)
   {
      return volatileInputLong(label, imLong, 1);
   }

   public static boolean volatileInputLong(String label, ImLong imLong, long step)
   {
      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags += ImGuiInputTextFlags.EnterReturnsTrue;
      return ImGui.inputScalar(label, ImGuiDataType.U32, imLong, step, 100, "%d", inputTextFlags);
   }

   public static boolean volatileInputFloat(String label, ImFloat imFloat)
   {
      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags += ImGuiInputTextFlags.EnterReturnsTrue;
      return ImGui.inputFloat(label, imFloat, 0, 0, "%.3f", inputTextFlags);
   }

   public static boolean volatileInputFloat(String label, ImFloat imFloat, float step)
   {
      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags += ImGuiInputTextFlags.EnterReturnsTrue;
      return ImGui.inputFloat(label, imFloat, step, 0, "%.3f", inputTextFlags);
   }

   public static boolean volatileInputDouble(String label, ImDouble imDouble)
   {
      return volatileInputDouble(label, imDouble, 0, 0);
   }

   public static boolean volatileInputDouble(String label, ImDouble imDouble, double step, double stepFast)
   {
      return volatileInputDouble(label, imDouble, step, stepFast, "%.6f");
   }

   public static boolean volatileInputDouble(String label, ImDouble imDouble, double step, double stepFast, String format)
   {
      int inputTextFlags = ImGuiInputTextFlags.None;
      inputTextFlags += ImGuiInputTextFlags.EnterReturnsTrue;
      return ImGui.inputDouble(label, imDouble, step, stepFast, format, inputTextFlags);
   }

   public static boolean sliderDouble(String label, ImDouble imDouble, double minValue, double maxValue)
   {
      return ImGui.sliderScalar(label, ImGuiDataType.Double, imDouble, minValue, maxValue);
   }

   public static boolean sliderDouble(String label, ImDouble imDouble, double minValue, double maxValue, String format)
   {
      return ImGui.sliderScalar(label, ImGuiDataType.Double, imDouble, minValue, maxValue, format);
   }

   public static boolean sliderDouble(String label, ImDouble imDouble, double minValue, double maxValue, String format, int imGuiSliderFlags)
   {
      return ImGui.sliderScalar(label, ImGuiDataType.Double, imDouble, minValue, maxValue, format, imGuiSliderFlags);
   }

   public static boolean sliderInt(String label, ImInt imInt, int minValue, int maxValue)
   {
      return ImGui.sliderScalar(label, ImGuiDataType.U32, imInt, minValue, maxValue);
   }

   public static boolean sliderInt(String label, ImInt imInt, int minValue, int maxValue, String format)
   {
      return ImGui.sliderScalar(label, ImGuiDataType.U32, imInt, minValue, maxValue, format);
   }

   public static boolean sliderInt(String label, ImInt imInt, int minValue, int maxValue, String format, int imGuiSliderFlags)
   {
      return ImGui.sliderScalar(label, ImGuiDataType.U32, imInt, minValue, maxValue, format, imGuiSliderFlags);
   }

   /**
    * Returns true if the user presses Enter, but unlike the EnterReturnsTrue flag,
    * using this method, the currently input text can be retrieved without the
    * user hitting the Enter key.
    *
    * @return if the user presses Enter
    */
   public static boolean inputText(String label, ImString text)
   {
      ImGui.inputText(label, text);
      return ImGui.isItemFocused() && ImGui.isKeyReleased(ImGuiTools.getEnterKey());
   }

   public static void textColored(Color color, String text)
   {
      ImGui.textColored(color.r, color.g, color.b, color.a, text);
   }

   public static void textBold(String text)
   {
      ImGui.pushFont(getSmallBoldFont());
      ImGui.text(text);
      ImGui.popFont();
   }

   public static void previousWidgetTooltip(String tooltipText)
   {
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip(tooltipText);
      }
   }

   /**
    * Places a mark, a vertical black line, at some point on the progress bar to
    * convey to the user where a threshold is.
    */
   public static void markedProgressBar(float barHeight, float barWidth, int color, double percent, double markPercent, String text)
   {
      float markPosition = (float) (barWidth * markPercent);
      float actualCursorX = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      float actualCursorY = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();
      float verticalExtents = 3.0f;
      ImGui.getWindowDrawList().addRectFilled(actualCursorX + markPosition,
                                              actualCursorY - verticalExtents,
                                              actualCursorX + markPosition + 2.0f,
                                              actualCursorY + barHeight + verticalExtents,
                                              ImGuiTools.BLACK, 1.0f);
      ImGui.pushStyleColor(ImGuiCol.PlotHistogram, color);
      ImGui.progressBar((float) percent, barWidth, barHeight, text);
      ImGui.popStyleColor();
   }

   /**
    * Useful for custom widgets.
    * @return Whether the area of the current custom item is hovered.
    */
   public static boolean isItemHovered(float itemWidth)
   {
      float mousePosXInDesktopFrame = ImGui.getMousePosX();
      float mousePosYInDesktopFrame = ImGui.getMousePosY();
      // Widget frame is the top-left of the start of the widgets, which is not the same as window
      // frame in the case the window is scrolled.
      float mousePosXInWidgetFrame = mousePosXInDesktopFrame - ImGui.getWindowPosX() + ImGui.getScrollX();
      float mousePosYInWidgetFrame = mousePosYInDesktopFrame - ImGui.getWindowPosY() + ImGui.getScrollY();

      boolean isHovered = mousePosXInWidgetFrame >= ImGui.getCursorPosX();
      isHovered &= mousePosXInWidgetFrame <= ImGui.getCursorPosX() + itemWidth + ImGui.getStyle().getFramePaddingX();
      isHovered &= mousePosYInWidgetFrame >= ImGui.getCursorPosY();
      isHovered &= mousePosYInWidgetFrame <= ImGui.getCursorPosY() + ImGui.getFontSize() + ImGui.getStyle().getFramePaddingY();
      isHovered &= ImGui.isWindowHovered();

      return isHovered;
   }

   /** ImGui doesn't support underlined text so this is the best we can do. */
   public static boolean textWithUnderlineOnHover(String text)
   {
      ImGui.calcTextSize(calcTextSize, text);

      // We must store the cursor position before rendering the text
      float cursorPosXInDesktopFrame = ImGui.getCursorScreenPosX();
      float cursorPosYInDesktopFrame = ImGui.getCursorScreenPosY();

      ImGui.text(text);

      boolean isHovered = ImGui.isItemHovered();

      if (isHovered)
      {
         ImGui.getWindowDrawList()
              .addRectFilled(cursorPosXInDesktopFrame,
                             cursorPosYInDesktopFrame + calcTextSize.y,
                             cursorPosXInDesktopFrame + calcTextSize.x,
                             cursorPosYInDesktopFrame + calcTextSize.y + 1.0f,
                             ImGui.getColorU32(ImGuiCol.Text));
      }

      return isHovered;
   }

   public static float calcTextSizeX(String text)
   {
      ImGui.calcTextSize(calcTextSize, text);
      return calcTextSize.x;
   }

   /** @deprecated Use ImGuiUniqueLabelMap instead. */
   public static String uniqueLabel(String label)
   {
      return label + "###GlobalWidgetIndex:" + nextWidgetIndex() + ":" + label;
   }

   public static String uniqueLabel(String id, String label)
   {
      return label + "###" + id + ":" + label;
   }

   /** @deprecated Use ImGuiUniqueLabelMap instead. */
   public static String uniqueLabel(Object thisObject, String label)
   {
      return label + "###" + thisObject.getClass().getName() + ":" + label;
   }

   /** @deprecated Use ImGuiUniqueLabelMap instead. */
   public static String uniqueIDOnly(Object thisObject, String label)
   {
      return "###" + thisObject.getClass().getName() + ":" + label;
   }

   public static ImFont setupFonts(ImGuiIO io)
   {
      return setupFonts(io, 1);
   }

   /**
    * See https://github.com/ocornut/imgui/blob/master/docs/FONTS.md
    * and ImGuiGlfwFreeTypeDemo in this project
    */
   public static ImFont setupFonts(ImGuiIO io, int fontSizeLevel)
   {
      final ImFontConfig smallFontConfig = new ImFontConfig(); // Natively allocated object, should be explicitly destroyed
      final ImFontConfig smallBoldFontConfig = new ImFontConfig();
      final ImFontConfig consoleFontConfig = new ImFontConfig();
      final ImFontConfig mediumFontConfig = new ImFontConfig();
      final ImFontConfig bigFontConfig = new ImFontConfig();
      final ImFontConfig nodeFontConfig = new ImFontConfig();

      // Glyphs could be added per-font as well as per config used globally like here
//      fontConfig.setGlyphRanges(fontAtlas.getGlyphRangesCyrillic());

//      fontConfig.setMergeMode(true); // When enabled, all fonts added with this config would be merged with the previously added font
      float size = 14.0f;
//      fontConfig.setSizePixels(size);
//      fontConfig.setOversampleH(4);
//      fontConfig.setOversampleV(4);
      int fontsFlags = 0;
      fontsFlags += ImGuiFreeTypeBuilderFlags.LightHinting;
//      fontConfig.setRasterizerFlags(flags);
//      fontConfig.setRasterizerMultiply(2.0f);
//      fontConfig.setPixelSnapH(true);
      smallFontConfig.setFontBuilderFlags(fontsFlags);
      smallBoldFontConfig.setFontBuilderFlags(fontsFlags + ImGuiFreeTypeBuilderFlags.Bold);
      consoleFontConfig.setFontBuilderFlags(fontsFlags);
      mediumFontConfig.setFontBuilderFlags(fontsFlags);
      bigFontConfig.setFontBuilderFlags(fontsFlags);
      nodeFontConfig.setFontBuilderFlags(fontsFlags);

//      fontToReturn = fontAtlas.addFontDefault(); // Add a default font, which is 'ProggyClean.ttf, 13px'
//      fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("basis33.ttf"), 16, fontConfig);
      String fontDirectory;
      if (SystemUtils.IS_OS_WINDOWS)
         fontDirectory = System.getenv("WINDIR") + "/Fonts";
      else
         fontDirectory = "/usr/share/fonts/TTF/";

      Path segoeui = Paths.get(fontDirectory, "segoeui.ttf");
      if (Files.exists(segoeui))
      {
         smallFontConfig.setName("segoeui.ttf, 16px");
         smallFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 16.0f, smallFontConfig);
         smallBoldFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 16.0f, smallBoldFontConfig);

         mediumFontConfig.setName("segoeui.ttf, 20px");
         mediumFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 20.0f, mediumFontConfig);

         bigFontConfig.setName("segoeui.ttf, 38px");
         bigFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 38.0f, bigFontConfig);

         nodeFontConfig.setName("segoeui.ttf, 32px 1/2");
         nodeFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 32.0f, nodeFontConfig);
      }
      else
      {
         smallFontConfig.setName("DejaVuSans.ttf, 13px");
         smallFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 13.0f, smallFontConfig);
         smallBoldFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 13.0f, smallBoldFontConfig);

         mediumFontConfig.setName("DejaVuSans.ttf, 17px");
         mediumFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 17.0f, mediumFontConfig);

         bigFontConfig.setName("DejaVuSans.ttf, 32px");
         bigFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 32.0f, bigFontConfig);

         nodeFontConfig.setName("DejaVuSans.ttf, 26px 1/2");
         nodeFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 26.0f, nodeFontConfig);

         // Accomodate for ImGui issue where the Windows fonts will render smaller than normal
         // This is so saving layout does not change the result depending on the fonts you have installed.
         // Can be removed when this is fixed: https://github.com/ocornut/imgui/issues/4780
         ImGui.getStyle().setFramePadding(ImGui.getStyle().getFramePaddingX(), 4.5f);
      }
      Path lucidaConsole = Paths.get(fontDirectory, "lucon.ttf");

      ImFontGlyphRangesBuilder glyphRangesBuilder = new ImFontGlyphRangesBuilder();
      glyphRangesBuilder.addRanges(ImGui.getIO().getFonts().getGlyphRangesDefault());
      glyphRangesBuilder.addRanges(new short[] {0x2500, 0x257F, 0}); // Box drawing https://www.compart.com/en/unicode/block/U+2500
      glyphRangesBuilder.addRanges(new short[] {0x2580, 0x259F, 0}); // Block elements https://www.compart.com/en/unicode/block/U+2580
      glyphRangesBuilder.addRanges(new short[] {0x25A0, 0x25FF, 0}); // Geometric shapes https://www.compart.com/en/unicode/block/U+25A0
      short[] glyphRanges = glyphRangesBuilder.buildRanges();

      if (Files.exists(lucidaConsole))
      {
         consoleFontConfig.setName("lucon.ttf, 12px");
         consoleFont = io.getFonts().addFontFromFileTTF(lucidaConsole.toAbsolutePath().toString(), 12.0f, consoleFontConfig, glyphRanges);
      }
      else
      {
         consoleFontConfig.setName("dejaVu/DejaVuSansMono.ttf, 12px");
         consoleFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSansMono.ttf"), 12.0f, consoleFontConfig, glyphRanges);
      }
//      fontConfig.setName("Roboto-Regular.ttf, 14px"); // This name will be displayed in Style Editor
//      fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), size, fontConfig);
//      fontConfig.setName("Roboto-Regular.ttf, 16px"); // We can apply a new config value every time we add a new font
//      fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), 16, fontConfig);

//      fontConfig.setName("Segoe UI"); // We can apply a new config value every time we add a new font
//      fontToReturn = fontAtlas.addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", size, fontConfig);

      nodeFont.setScale(0.5f);

      fontAtlas = ImGui.getIO().getFonts();
      fontAtlas.build();

      smallFontConfig.destroy(); // After all fonts were added we don't need this config more
      consoleFontConfig.destroy();
      mediumFontConfig.destroy();
      bigFontConfig.destroy();
      nodeFontConfig.destroy();

      if (fontSizeLevel == 2)
         return mediumFont;
      if (fontSizeLevel == 3)
         return bigFont;

      return smallFont;
   }

   public static ImFont getBigFont() {
      return bigFont;
   }

   public static ImFont getMediumFont()
   {
      return mediumFont;
   }

   public static ImFont getSmallFont()
   {
      return smallFont;
   }

   public static ImFont getSmallBoldFont()
   {
      return smallBoldFont;
   }

   public static ImFont getNodeFont() {
      return nodeFont;
   }

   public static ImFont getConsoleFont()
   {
      return consoleFont;
   }

   public static ImFontAtlas getFontAtlas()
   {
      return fontAtlas;
   }

   public static byte[] loadFromResources(final String fileName)
   {
      try (InputStream is = Objects.requireNonNull(ImGuiTools.class.getClassLoader().getResourceAsStream(fileName));
           ByteArrayOutputStream buffer = new ByteArrayOutputStream())
      {

         final byte[] data = new byte[16384];

         int nRead;
         while ((nRead = is.read(data, 0, data.length)) != -1)
         {
            buffer.write(data, 0, nRead);
         }

         return buffer.toByteArray();
      }
      catch (IOException e)
      {
         throw new UncheckedIOException(e);
      }
   }

   public static BoundingBox2D windowBoundingBox()
   {
      BoundingBox2D box = new BoundingBox2D();
      int posX = (int) ImGui.getWindowPosX();
      int posY = (int) ImGui.getWindowPosY();
      box.setMin(posX, posY);
      box.setMax(posX + ImGui.getWindowSizeX(), posY + (int) ImGui.getWindowSizeX());
      return box;
   }

   public static float getUsableWindowWidth()
   {
      return ImGui.getWindowWidth() - 32.0f;
   }

   private static void initializeUserMappedKeys()
   {
      spaceKey = ImGui.getKeyIndex(ImGuiKey.Space);
      deleteKey = ImGui.getKeyIndex(ImGuiKey.Delete);
      escapeKey = ImGui.getKeyIndex(ImGuiKey.Escape);
      enterKey = ImGui.getKeyIndex(ImGuiKey.Enter);
      upArrowKey = ImGui.getKeyIndex(ImGuiKey.UpArrow);
      downArrowKey = ImGui.getKeyIndex(ImGuiKey.DownArrow);
      leftArrowKey = ImGui.getKeyIndex(ImGuiKey.LeftArrow);
      rightArrowKey = ImGui.getKeyIndex(ImGuiKey.RightArrow);
   }

   public static int getSpaceKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return spaceKey;
   }

   public static int getDeleteKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return deleteKey;
   }

   public static int getEscapeKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return escapeKey;
   }

   public static int getEnterKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return enterKey;
   }

   public static int getUpArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return upArrowKey;
   }

   public static int getDownArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return downArrowKey;
   }

   public static int getLeftArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return leftArrowKey;
   }

   public static int getRightArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return rightArrowKey;
   }
}
