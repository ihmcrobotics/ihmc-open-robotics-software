package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class GDX2DSceneManager
{
   private final ArrayList<GDX2DSprite> sprites = new ArrayList<>();
   private SpriteBatch spriteBatch;

   private InputMultiplexer inputMultiplexer;
   private GDX2DOrthographicCamera orthographicCamera;
   private ScreenViewport viewport;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private Runnable onCreate;
   private GLProfiler glProfiler;

   public void create()
   {
      create(GDXInputMode.libGDX);
   }

   public void create(GDXInputMode inputMode)
   {
      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = GDXTools.createGLProfiler();

      GDXTools.syncLogLevelWithLogTools();

      orthographicCamera = new GDX2DOrthographicCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(orthographicCamera.setInputForLibGDX());
      }

      spriteBatch = new SpriteBatch();

      viewport = new ScreenViewport(orthographicCamera);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      if (onCreate != null)
         onCreate.run();
   }

   public void render()
   {
      preRender();
      for (GDX2DSprite sprite : sprites)
      {
         sprite.draw(spriteBatch);
      }
      spriteBatch.end();

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler.reset();
   }

   private void preRender()
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");
      }

      if (width < 0)
         width = getCurrentWindowWidth();
      if (height < 0)
         height = getCurrentWindowHeight();

      viewport.update(width, height);

      spriteBatch.begin();

      GL41.glViewport(x, y, width, height);
      GDX3DSceneTools.glClearGray();
   }

   public void dispose()
   {

   }
   // End render public API

   public boolean closeRequested()
   {
      return true;
   }

   public void setViewportBoundsToWindow()
   {
      setViewportBounds(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
   }

   /**
    * Coordinates in xy bottom left
    */
   public void setViewportBounds(int x, int y, int width, int height)
   {
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;
   }

   public int getCurrentWindowWidth()
   {
      return Gdx.graphics.getWidth();
   }

   public int getCurrentWindowHeight()
   {
      return Gdx.graphics.getHeight();
   }

   public GDX2DOrthographicCamera getOrthographicCamera()
   {
      return orthographicCamera;
   }

   public void addLibGDXInputProcessor(InputProcessor inputProcessor)
   {
      if (inputMultiplexer != null)
      {
         inputMultiplexer.addProcessor(inputProcessor);
      }
      else
      {
         LogTools.error(1, "libGDX is not being used for input!");
      }
   }

   public void setOnCreate(Runnable onCreate)
   {
      this.onCreate = onCreate;
   }

   public ArrayList<GDX2DSprite> getSprites()
   {
      return sprites;
   }
}
