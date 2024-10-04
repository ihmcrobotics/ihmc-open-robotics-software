package us.ihmc.rdx.vr.minimalLibGDX;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Graphics;
import com.badlogic.gdx.graphics.Cursor;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.GL30;
import com.badlogic.gdx.graphics.GL31;
import com.badlogic.gdx.graphics.GL32;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.GLVersion;

public class MinimalLibGDXGraphics implements Graphics
{
   @Override
   public boolean isGL30Available()
   {
      return true;
   }

   @Override
   public boolean isGL31Available()
   {
      return true;
   }

   @Override
   public boolean isGL32Available()
   {
      return true;
   }

   @Override
   public GL20 getGL20()
   {
      return Gdx.gl20;
   }

   @Override
   public GL30 getGL30()
   {
      return Gdx.gl30;
   }

   @Override
   public GL31 getGL31()
   {
      return Gdx.gl31;
   }

   @Override
   public GL32 getGL32()
   {
      return Gdx.gl32;
   }

   @Override
   public void setGL20(GL20 gl20)
   {

   }

   @Override
   public void setGL30(GL30 gl30)
   {

   }

   @Override
   public void setGL31(GL31 gl31)
   {

   }

   @Override
   public void setGL32(GL32 gl32)
   {

   }

   @Override
   public int getWidth()
   {
      return 0;
   }

   @Override
   public int getHeight()
   {
      return 0;
   }

   @Override
   public int getBackBufferWidth()
   {
      return 0;
   }

   @Override
   public int getBackBufferHeight()
   {
      return 0;
   }

   @Override
   public float getBackBufferScale()
   {
      return 0;
   }

   @Override
   public int getSafeInsetLeft()
   {
      return 0;
   }

   @Override
   public int getSafeInsetTop()
   {
      return 0;
   }

   @Override
   public int getSafeInsetBottom()
   {
      return 0;
   }

   @Override
   public int getSafeInsetRight()
   {
      return 0;
   }

   @Override
   public long getFrameId()
   {
      return 0;
   }

   @Override
   public float getDeltaTime()
   {
      return 0;
   }

   @Override
   public float getRawDeltaTime()
   {
      return 0;
   }

   @Override
   public int getFramesPerSecond()
   {
      return 0;
   }

   @Override
   public GraphicsType getType()
   {
      return null;
   }

   @Override
   public GLVersion getGLVersion()
   {
      return null;
   }

   @Override
   public float getPpiX()
   {
      return 0;
   }

   @Override
   public float getPpiY()
   {
      return 0;
   }

   @Override
   public float getPpcX()
   {
      return 0;
   }

   @Override
   public float getPpcY()
   {
      return 0;
   }

   @Override
   public float getDensity()
   {
      return 0;
   }

   @Override
   public boolean supportsDisplayModeChange()
   {
      return false;
   }

   @Override
   public Monitor getPrimaryMonitor()
   {
      return null;
   }

   @Override
   public Monitor getMonitor()
   {
      return null;
   }

   @Override
   public Monitor[] getMonitors()
   {
      return new Monitor[0];
   }

   @Override
   public DisplayMode[] getDisplayModes()
   {
      return new DisplayMode[0];
   }

   @Override
   public DisplayMode[] getDisplayModes(Monitor monitor)
   {
      return new DisplayMode[0];
   }

   @Override
   public DisplayMode getDisplayMode()
   {
      return null;
   }

   @Override
   public DisplayMode getDisplayMode(Monitor monitor)
   {
      return null;
   }

   @Override
   public boolean setFullscreenMode(DisplayMode displayMode)
   {
      return false;
   }

   @Override
   public boolean setWindowedMode(int width, int height)
   {
      return false;
   }

   @Override
   public void setTitle(String title)
   {

   }

   @Override
   public void setUndecorated(boolean undecorated)
   {

   }

   @Override
   public void setResizable(boolean resizable)
   {

   }

   @Override
   public void setVSync(boolean vsync)
   {

   }

   @Override
   public void setForegroundFPS(int fps)
   {

   }

   @Override
   public BufferFormat getBufferFormat()
   {
      return null;
   }

   @Override
   public boolean supportsExtension(String extension)
   {
      return true;
   }

   @Override
   public void setContinuousRendering(boolean isContinuous)
   {

   }

   @Override
   public boolean isContinuousRendering()
   {
      return false;
   }

   @Override
   public void requestRendering()
   {

   }

   @Override
   public boolean isFullscreen()
   {
      return false;
   }

   @Override
   public Cursor newCursor(Pixmap pixmap, int xHotspot, int yHotspot)
   {
      return null;
   }

   @Override
   public void setCursor(Cursor cursor)
   {

   }

   @Override
   public void setSystemCursor(Cursor.SystemCursor systemCursor)
   {

   }
}
