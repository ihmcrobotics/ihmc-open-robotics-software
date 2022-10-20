package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.rdx.input.GDXInputMode;
import us.ihmc.rdx.tools.GDXTools;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

public class GDX2DSceneManager
{
   private final ArrayList<GDX2DSpriteDrawable> sprites = new ArrayList<>();
   private SpriteBatch spriteBatch;
   private final ArrayList<Consumer<ArrayList<GDX2DSpriteDrawable>>> spriteRenderableProviders = new ArrayList<>();
   private final ConcurrentLinkedQueue<Consumer<ArrayList<GDX2DSpriteDrawable>>> spriteRenderableRemovalQueue = new ConcurrentLinkedQueue();

   private InputMultiplexer inputMultiplexer;
   private GDX2DOrthographicCamera orthographicCamera;
   private ScreenViewport screenViewport;

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

      spriteBatch = new SpriteBatch();

      orthographicCamera = new GDX2DOrthographicCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(orthographicCamera.setInputForLibGDX());
      }

      screenViewport = new ScreenViewport(orthographicCamera);
      screenViewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?
      screenViewport.apply();

      if (onCreate != null)
         onCreate.run();
   }

   public void render()
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");
      }

      while (!spriteRenderableRemovalQueue.isEmpty())
         spriteRenderableProviders.remove(spriteRenderableRemovalQueue.poll());

      screenViewport.update(width, height);

      GDX3DSceneTools.glClearGray();

      spriteBatch.setProjectionMatrix(orthographicCamera.combined);
      spriteBatch.begin();

      sprites.clear();
      for (Consumer<ArrayList<GDX2DSpriteDrawable>> spriteRenderableProvider : spriteRenderableProviders)
      {
         spriteRenderableProvider.accept(sprites);
      }

      for (GDX2DSpriteDrawable sprite : sprites)
      {
         sprite.draw(spriteBatch);
      }

      spriteBatch.end();

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler.reset();
   }

   // End render public API

   public boolean closeRequested()
   {
      return true;
   }

   /**
    * Coordinates in xy bottom left
    */
   public void setViewportBounds(int width, int height)
   {
      this.width = width;
      this.height = height;
   }

   public GDX2DOrthographicCamera getOrthographicCamera()
   {
      return orthographicCamera;
   }

   public ScreenViewport getScreenViewport()
   {
      return screenViewport;
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

   public void addSpriteRenderableProvider(Consumer<ArrayList<GDX2DSpriteDrawable>> spriteRenderableProvider)
   {
      spriteRenderableProviders.add(spriteRenderableProvider);
   }

   public void queueRemoveSpriteRenderableProvider(Consumer<ArrayList<GDX2DSpriteDrawable>> spriteRenderableProvider)
   {
      spriteRenderableRemovalQueue.add(spriteRenderableProvider);
   }
}
