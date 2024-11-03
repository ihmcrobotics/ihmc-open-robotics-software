package us.ihmc.rdx.imgui;

import com.badlogic.gdx.Input;
import com.badlogic.gdx.graphics.Color;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import imgui.ImFont;
import imgui.ImFontAtlas;
import imgui.ImFontConfig;
import imgui.ImFontGlyphRangesBuilder;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiDataType;
import imgui.flag.ImGuiFreeTypeBuilderFlags;
import imgui.flag.ImGuiInputTextFlags;
import imgui.flag.ImGuiKey;
import imgui.internal.ImGuiContext;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import imgui.type.ImLong;
import imgui.type.ImString;
import org.apache.commons.lang3.SystemUtils;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.log.LogTools;
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

   private static String FONT_DIRECTORY;
   public static int SMALLEST_FONT_SIZE = 9;
   public static int DEFAULT_FONT_SIZE = 13;
   public static int LARGEST_FONT_SIZE = 26;
   public static int CURRENT_FONT_SIZE = DEFAULT_FONT_SIZE;
   private static Path SEGOE_UI_PATH;
   private static Path LUCIDA_CONSOLE_PATH;
   public static boolean SEGOE_EXISTS;
   public static boolean LUCIDA_EXISTS;
   public static final double DEJAVU_TO_SEGOE_SCALE = 1.2;
   public static final double SMALL_TO_MEDIUM_SCALE = 1.25;
   public static final double SMALL_TO_LARGE_SCALE = 2.4;

   private static final TIntObjectMap<ImFont> consoleFont = new TIntObjectHashMap<>();
   private static final TIntObjectMap<ImFont> smallFont = new TIntObjectHashMap<>();
   private static final TIntObjectMap<ImFont> smallBoldFont = new TIntObjectHashMap<>();
   private static final TIntObjectMap<ImFont> mediumFont = new TIntObjectHashMap<>();
   private static final TIntObjectMap<ImFont> bigFont = new TIntObjectHashMap<>();

   private static boolean userKeysHaveBeenMapped = false;
   private static int spaceKey;
   private static int deleteKey;
   private static int escapeKey;
   private static int enterKey;
   private static int keyPadEnterKey;
   private static int upArrowKey;
   private static int downArrowKey;
   private static int leftArrowKey;
   private static int rightArrowKey;
   private static ImFontAtlas fontAtlas;

   public static int BLACK = Color.BLACK.toIntBits();
   public static int WHITE = Color.WHITE.toIntBits();
   public static int GRAY = Color.GRAY.toIntBits();
   public static int RED = Color.RED.toIntBits();
   public static int CYAN = Color.CYAN.toIntBits();
   public static int PURPLE = new Color(1.0f, 0.0f, 1.0f, 1.0f).toIntBits();
   public static int YELLOW = Color.YELLOW.toIntBits();
   public static int GREEN = Color.GREEN.toIntBits();
   public static int DARK_RED = new Color(0.7f, 0.0f, 0.0f, 1.0f).toIntBits();
   public static int DARK_PURPLE = new Color(0.7f, 0.0f, 0.7f, 1.0f).toIntBits();
   public static int DARK_GREEN = new Color(0.0f, 0.7f, 0.0f, 1.0f).toIntBits();
   public static int DARK_ORANGE = new Color(1.0f, 0.55f, 0.0f, 1.0f).toIntBits();
   public static int LIGHT_GRAY = Color.LIGHT_GRAY.toIntBits();
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
    * Method for getting color ranging from green to red based on an input value,
    * and the values at which the input should result in green and red.
    *
    * @param value The value which determines returned color
    * @param valueAtGreen The value at which returned color will be green
    * @param valueAtRed The value at which returned color will be red
    * @return Integer value representing color
    */
   public static int greenRedGradientColor(double value, double valueAtGreen, double valueAtRed)
   {
      double valueRange = valueAtGreen - valueAtRed;
      double greenStrength = MathTools.clamp((value - valueAtRed) / valueRange, 0.0, 1.0);
      double redStrength = 1.0 - greenStrength;
      return new Color((float) redStrength, (float) greenStrength, 0.0f, 1.0f).toIntBits();
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

   public static boolean smallCheckbox(String label, ImBoolean checked)
   {
      float backupFramePaddingX = ImGui.getStyle().getFramePaddingX();
      float backupFramePaddingY = ImGui.getStyle().getFramePaddingY();
      ImGui.getStyle().setFramePadding(backupFramePaddingX, 0.0f);
      boolean pressed = ImGui.checkbox(label, checked);
      ImGui.getStyle().setFramePadding(backupFramePaddingX, backupFramePaddingY);
      return pressed;
   }

   public static boolean smallCheckbox(String label, boolean checked)
   {
      float backupFramePaddingX = ImGui.getStyle().getFramePaddingX();
      float backupFramePaddingY = ImGui.getStyle().getFramePaddingY();
      ImGui.getStyle().setFramePadding(backupFramePaddingX, 0.0f);
      boolean pressed = ImGui.checkbox(label, checked);
      ImGui.getStyle().setFramePadding(backupFramePaddingX, backupFramePaddingY);
      return pressed;
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
      return ImGui.isItemFocused() && (ImGui.isKeyReleased(ImGuiTools.getEnterKey()) || ImGui.isKeyReleased(ImGuiTools.getKeyPadEnterKey()));
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
      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      float verticalExtents = 3.0f;
      float notchWidth = 2.0f;
      ImGui.getWindowDrawList().addRectFilled(cursorScreenPosX + markPosition,
                                              cursorScreenPosY - verticalExtents,
                                              cursorScreenPosX + markPosition + notchWidth,
                                              cursorScreenPosY + barHeight + verticalExtents,
                                              ImGuiTools.BLACK);
      ImGui.pushStyleColor(ImGuiCol.PlotHistogram, color);
      ImGui.progressBar((float) percent, barWidth, barHeight, text);
      ImGui.popStyleColor();
   }

   public static void renderSliderOrProgressNotch(float x, int color)
   {
      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      float verticalExtents = 3.0f;
      float notchWidth = 2.0f;

      ImGui.getWindowDrawList().addRectFilled(cursorScreenPosX + x,
                                              ImGui.getCursorScreenPosY() - verticalExtents,
                                              cursorScreenPosX + x + notchWidth,
                                              cursorScreenPosY + ImGui.getFrameHeight() + verticalExtents,
                                              color);
   }

   /**
    * Useful for custom widgets.
    * @param itemWidth the width of the applicable area after the cursor
    * @param lineHeight Needs to get passed in because it could be either
    *                  {@link ImGui#getTextLineHeight} or {@link ImGui#getFrameHeight}
    * @return Whether the area of the current custom item is hovered.
    */
   public static boolean isItemHovered(float itemWidth, float lineHeight)
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
      isHovered &= mousePosYInWidgetFrame <= ImGui.getCursorPosY() + lineHeight;
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

   /**
    * A separator with a label in it of given font.
    * This is in the newer versions of ImGui.
    */
   public static void separatorText(String text, ImFont font)
   {
      ImGui.pushFont(font);
      ImGuiTools.separatorText(text);
      ImGui.popFont();
   }

   /**
    * A separator with a label in it.
    * This is in the newer versions of ImGui.
    */
   public static void separatorText(String text)
   {
      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      int fontSize = ImGui.getFontSize();
      float itemSpacingX = ImGui.getStyle().getItemSpacingX();
      float lineThickness = fontSize * 0.2f;
      float lineY = ImGui.getTextLineHeight() / 2.0f + (lineThickness / 2.0f);
      float initialLineWidth = fontSize * 1.5f;
      int separatorColor = ImGui.getColorU32(ImGuiCol.Separator);
      ImGui.getWindowDrawList().addLine(cursorScreenPosX, cursorScreenPosY + lineY,
                                        cursorScreenPosX + initialLineWidth, cursorScreenPosY + lineY,
                                        separatorColor, lineThickness);
      ImGui.setCursorPosX(ImGui.getCursorPosX() + initialLineWidth + itemSpacingX);
      ImGui.text(text);
      ImGui.sameLine();
      float startX = ImGui.getCursorPosX() - itemSpacingX;
      float endX = ImGui.getContentRegionMaxX();
      float secondLineWidth = endX - startX;
      ImGui.getWindowDrawList().addLine(cursorScreenPosX + startX, cursorScreenPosY + lineY,
                                        cursorScreenPosX + endX, cursorScreenPosY + lineY,
                                        separatorColor, lineThickness);
      ImGui.setCursorPosX(ImGui.getCursorPosX() + secondLineWidth);
      ImGui.newLine();
   }

   public static void rightAlignText(String text)
   {
      float windowWidth = ImGui.getWindowContentRegionMaxX();
      float textWidth = calcTextSizeX(text);
      float cursorPosX = ImGui.getCursorPosX();
      ImGui.setCursorPosX(windowWidth - textWidth);
      ImGui.text(text);
      ImGui.setCursorPosY(cursorPosX);
   }

   public static float calcTextSizeX(String text)
   {
      ImGui.calcTextSize(calcTextSize, text);
      return calcTextSize.x;
   }

   public static float calcTextSizeY(String text)
   {
      ImGui.calcTextSize(calcTextSize, text);
      return calcTextSize.y;
   }

   public static float calcButtonWidth(String buttonText)
   {
      return calcTextSizeX(buttonText) + ImGui.getStyle().getFrameBorderSize() * 2 + ImGui.getStyle().getItemInnerSpacingX() * 2;
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

   /**
    * See <a href="https://github.com/ocornut/imgui/blob/master/docs/FONTS.md">FONTS.md</a>
    * and ImGuiGlfwFreeTypeDemo in this project
    */
   public static void setupFonts(ImGuiIO io)
   {
      if (SystemUtils.IS_OS_WINDOWS)
         FONT_DIRECTORY = System.getenv("WINDIR") + "/Fonts";
      else
         FONT_DIRECTORY = "/usr/share/fonts/TTF/";

      ImFontGlyphRangesBuilder glyphRangesBuilder = new ImFontGlyphRangesBuilder();
      glyphRangesBuilder.addRanges(ImGui.getIO().getFonts().getGlyphRangesDefault());
      glyphRangesBuilder.addRanges(new short[] {0x2500, 0x257F, 0}); // Box drawing https://www.compart.com/en/unicode/block/U+2500
      glyphRangesBuilder.addRanges(new short[] {0x2580, 0x259F, 0}); // Block elements https://www.compart.com/en/unicode/block/U+2580
      glyphRangesBuilder.addRanges(new short[] {0x25A0, 0x25FF, 0}); // Geometric shapes https://www.compart.com/en/unicode/block/U+25A0
      short[] glyphRanges = glyphRangesBuilder.buildRanges();

      SEGOE_UI_PATH = Paths.get(FONT_DIRECTORY, "segoeui.ttf");
      LUCIDA_CONSOLE_PATH = Paths.get(FONT_DIRECTORY, "lucon.ttf");
      SEGOE_EXISTS = Files.exists(SEGOE_UI_PATH);
      LUCIDA_EXISTS = Files.exists(LUCIDA_CONSOLE_PATH);

      if (!SEGOE_EXISTS)
      {
         // Accomodate for ImGui issue where the Windows fonts will render smaller than normal
         // This is so saving layout does not change the result depending on the fonts you have installed.
         // Can be removed when this is fixed: https://github.com/ocornut/imgui/issues/4780
         ImGui.getStyle().setFramePadding(ImGui.getStyle().getFramePaddingX(), 4.5f);
      }

      for (int i = SMALLEST_FONT_SIZE; i < LARGEST_FONT_SIZE + 1; i++)
      {
         int smallSize = (int) Math.round(SEGOE_EXISTS ? DEJAVU_TO_SEGOE_SCALE * i : i);
         int mediumSize = (int) Math.round(SEGOE_EXISTS ? DEJAVU_TO_SEGOE_SCALE * SMALL_TO_MEDIUM_SCALE * i : SMALL_TO_MEDIUM_SCALE * i);
         int largeSize = (int) Math.round(SEGOE_EXISTS ? DEJAVU_TO_SEGOE_SCALE * SMALL_TO_LARGE_SCALE * i : SMALL_TO_LARGE_SCALE * i);

         smallFont.put(i, loadFont(io, smallSize, 0, true, glyphRanges));
         smallBoldFont.put(i, loadFont(io, smallSize, ImGuiFreeTypeBuilderFlags.Bold, true, glyphRanges));
         consoleFont.put(i, loadFont(io, i - 1, 0, false, glyphRanges));
         mediumFont.put(i, loadFont(io, mediumSize, 0, true, glyphRanges));
         bigFont.put(i, loadFont(io, largeSize, 0, true, glyphRanges));
      }

      fontAtlas = ImGui.getIO().getFonts();
      fontAtlas.build();
   }

   private static ImFont loadFont(ImGuiIO io, int size, int addedFlags, boolean isSans, short[] glyphRanges)
   {
      ImFontConfig fontConfig = new ImFontConfig();

      // Glyphs could be added per-font as well as per config used globally like here
      // fontConfig.setGlyphRanges(fontAtlas.getGlyphRangesCyrillic());
      // fontConfig.setMergeMode(true); // When enabled, all fonts added with this config would be merged with the previously added font
      // fontConfig.setSizePixels(size);
      // fontConfig.setOversampleH(4);
      // fontConfig.setOversampleV(4);
      // fontConfig.setRasterizerFlags(flags);
      // fontConfig.setRasterizerMultiply(2.0f);
      // fontConfig.setPixelSnapH(true);

      int fontsFlags = 0;
      fontsFlags += ImGuiFreeTypeBuilderFlags.LightHinting;
      fontConfig.setFontBuilderFlags(fontsFlags + addedFlags);

      ImFont font;
      if (isSans && SEGOE_EXISTS)
      {
         fontConfig.setName("segoeui.ttf, %dpx".formatted(size));
         font = io.getFonts().addFontFromFileTTF(SEGOE_UI_PATH.toAbsolutePath().toString(), size, fontConfig);
      }
      else if (isSans)
      {
         fontConfig.setName("DejaVuSans.ttf, %dpx".formatted(size));
         font = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), size, fontConfig);
      }
      else if (LUCIDA_EXISTS)
      {
         fontConfig.setName("lucon.ttf, %dpx".formatted(size));
         font = io.getFonts().addFontFromFileTTF(LUCIDA_CONSOLE_PATH.toAbsolutePath().toString(), size, fontConfig, glyphRanges);
      }
      else
      {
         fontConfig.setName("dejaVu/DejaVuSansMono.ttf, %dpx".formatted(size));
         font = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSansMono.ttf"), size, fontConfig, glyphRanges);
      }

      // fontToReturn = fontAtlas.addFontDefault(); // Add a default font, which is 'ProggyClean.ttf, 13px'
      // fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("basis33.ttf"), 16, fontConfig);
      // fontConfig.setName("Roboto-Regular.ttf, 14px"); // This name will be displayed in Style Editor
      // fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), size, fontConfig);
      // fontConfig.setName("Roboto-Regular.ttf, 16px"); // We can apply a new config value every time we add a new font
      // fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), 16, fontConfig);
      // fontConfig.setName("Segoe UI"); // We can apply a new config value every time we add a new font
      // fontToReturn = fontAtlas.addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", size, fontConfig);

      fontConfig.destroy();

      return font;
   }

   public static ImFont getBigFont()
   {
      return bigFont.get(CURRENT_FONT_SIZE);
   }

   public static ImFont getMediumFont()
   {
      return mediumFont.get(CURRENT_FONT_SIZE);
   }

   public static ImFont getSmallFont()
   {
      return smallFont.get(CURRENT_FONT_SIZE);
   }

   public static ImFont getSmallBoldFont()
   {
      return smallBoldFont.get(CURRENT_FONT_SIZE);
   }

   private static boolean printedConsoleFontError = false;

   public static ImFont getConsoleFont()
   {
      // FIXME: The stuff below still doesn't work all the time.
      return getSmallFont();

//      ImFont font = consoleFont.get(CURRENT_FONT_SIZE);
//      if (!font.isLoaded()) // FIXME: Find issue and fix
//      {
//         if (!printedConsoleFontError)
//         {
//            printedConsoleFontError = true;
//            LogTools.error("Console font %s size %d not loaded!".formatted(font.getDebugName(), CURRENT_FONT_SIZE));
//         }
//
//         return getSmallFont();
//      }
//      else
//      {
//         return font;
//      }
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
      keyPadEnterKey = ImGui.getKeyIndex(ImGuiKey.KeyPadEnter);
      upArrowKey = ImGui.getKeyIndex(ImGuiKey.UpArrow);
      downArrowKey = ImGui.getKeyIndex(ImGuiKey.DownArrow);
      leftArrowKey = ImGui.getKeyIndex(ImGuiKey.LeftArrow);
      rightArrowKey = ImGui.getKeyIndex(ImGuiKey.RightArrow);
      userKeysHaveBeenMapped = true;
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

   public static int getKeyPadEnterKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return keyPadEnterKey;
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
