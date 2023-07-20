package us.ihmc.rdx;

import us.ihmc.log.LogTools;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Properties;
import java.util.concurrent.CompletableFuture;

public class RDXSettings
{
   static
   {
      deleteOldThemeFile();
   }

   private static int deleteRetries = 0;

   // We save this as .ini, but it's actually interpreted as standard Java Properties
   private static final Path RDX_SETTINGS_PATH = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("RDXSettings.ini");

   private boolean plotFrameRate = false;
   private boolean vsync = false;
   private int foregroundFPSLimit = 240;
   private int libGDXLogLevel = 2;
   private double imguiFontScale = 1.0;
   private String themeName = "LIGHT";
   private float view3DBackgroundShade = RDX3DSceneTools.CLEAR_COLOR;

   public boolean plotFrameRateEnabled()
   {
      return plotFrameRate;
   }

   public void setPlotFrameRate(boolean plotFrameRate)
   {
      this.plotFrameRate = plotFrameRate;
      saveAsync();
   }

   public boolean vsyncEnabled()
   {
      return vsync;
   }

   public void setVsync(boolean vsync)
   {
      this.vsync = vsync;
      saveAsync();
   }

   public int getForegroundFPSLimit()
   {
      return foregroundFPSLimit;
   }

   public void setForegroundFPSLimit(int foregroundFPSLimit)
   {
      this.foregroundFPSLimit = foregroundFPSLimit;
      saveAsync();
   }

   public int getLibGDXLogLevel()
   {
      return libGDXLogLevel;
   }

   public void setLibGDXLogLevel(int libGDXLogLevel)
   {
      this.libGDXLogLevel = libGDXLogLevel;
      saveAsync();
   }

   public double getImguiFontScale()
   {
      return imguiFontScale;
   }

   public void setImguiFontScale(double imguiFontScale)
   {
      this.imguiFontScale = imguiFontScale;
      saveAsync();
   }

   public String getThemeName()
   {
      return themeName;
   }

   public void setThemeName(String themeName)
   {
      this.themeName = themeName;
      saveAsync();
   }

   public float getView3DBackgroundShade()
   {
      return view3DBackgroundShade;
   }

   public void setView3DBackgroundShade(float view3DBackgroundShade)
   {
      this.view3DBackgroundShade = view3DBackgroundShade;
      saveAsync();
   }

   public void save() throws IOException
   {
      File file = RDX_SETTINGS_PATH.toFile();

      file.getParentFile().mkdirs();

      if (!file.exists())
      {
         file.createNewFile();
      }

      Properties properties = new Properties();
      properties.setProperty("plotFrameRate", String.valueOf(plotFrameRate));
      properties.setProperty("vsync", String.valueOf(vsync));
      properties.setProperty("foregroundFPSLimit", String.valueOf(foregroundFPSLimit));
      properties.setProperty("libgdxLogLevel", String.valueOf(libGDXLogLevel));
      properties.setProperty("imguiFontScale", String.valueOf(imguiFontScale));
      properties.setProperty("themeName", String.valueOf(themeName));
      properties.setProperty("view3DBackgroundShade", String.valueOf(view3DBackgroundShade));

      try (FileOutputStream output = new FileOutputStream(file))
      {
         properties.store(output, null);
      }
   }

   public void saveAsync()
   {
      CompletableFuture.runAsync(() ->
      {
         try
         {
            save();
         }
         catch (IOException e)
         {
            LogTools.info(e);
         }
      });
   }

   public void load() throws IOException
   {
      File file = RDX_SETTINGS_PATH.toFile();

      if (!file.exists())
      {
         save();
      }

      Properties properties = new Properties();

      try (FileInputStream input = new FileInputStream(file))
      {
         properties.load(input);
      }

      try
      {
         plotFrameRate = Boolean.parseBoolean(properties.getProperty("plotFrameRate"));
         vsync = Boolean.parseBoolean(properties.getProperty("vsync"));
         foregroundFPSLimit = Integer.parseInt(properties.getProperty("foregroundFPSLimit"));
         libGDXLogLevel = Integer.parseInt(properties.getProperty("libgdxLogLevel"));
         imguiFontScale = Double.parseDouble(properties.getProperty("imguiFontScale"));
         themeName = properties.getProperty("themeName");
         view3DBackgroundShade = Float.parseFloat(properties.getProperty("view3DBackgroundShade"));
      }
      catch (Exception e)
      {
         // Delete and re-create the file if there was a parsing error
         if (deleteRetries++ > 20)
            return; // Stop gap
         file.delete();
         save();
      }
   }

   private static void deleteOldThemeFile()
   {
      File oldThemeFile = new File(IHMCCommonPaths.DOT_IHMC_DIRECTORY.toFile(), "themePreference.ini");

      if (oldThemeFile.exists())
      {
         LogTools.info("Deleting old theme file: " + oldThemeFile.getAbsolutePath());
         oldThemeFile.delete();
      }
   }
}