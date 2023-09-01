package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.rdx.input.RDXInputMode;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

public class RDX2DSceneManager
{
   private final ArrayList<RDX2DSpriteDrawable> sprites = new ArrayList<>();
   private SpriteBatch spriteBatch;
   private final ArrayList<Consumer<ArrayList<RDX2DSpriteDrawable>>> spriteRenderableProviders = new ArrayList<>();
   private final ConcurrentLinkedQueue<Consumer<ArrayList<RDX2DSpriteDrawable>>> spriteRenderableRemovalQueue = new ConcurrentLinkedQueue();

   private InputMultiplexer inputMultiplexer;
   private RDX2DOrthographicCamera orthographicCamera;
   private ScreenViewport screenViewport;

   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private Runnable onCreate;
   private GLProfiler glProfiler;

   public void create()
   {
      create(RDXInputMode.libGDX);
   }

   public void create(RDXInputMode inputMode)
   {
      if (LibGDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = LibGDXTools.createGLProfiler();

      LibGDXTools.syncLogLevelWithLogTools();

      spriteBatch = new SpriteBatch();

      orthographicCamera = new RDX2DOrthographicCamera();
      if (inputMode == RDXInputMode.libGDX)
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

      RDX3DSceneTools.glClearGray();

      spriteBatch.setProjectionMatrix(orthographicCamera.combined);
      spriteBatch.begin();

      sprites.clear();
      for (Consumer<ArrayList<RDX2DSpriteDrawable>> spriteRenderableProvider : spriteRenderableProviders)
      {
         spriteRenderableProvider.accept(sprites);
      }

      for (RDX2DSpriteDrawable sprite : sprites)
      {
         sprite.draw(spriteBatch);
      }

      spriteBatch.end();

      if (LibGDXTools.ENABLE_OPENGL_DEBUGGER)
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

   public RDX2DOrthographicCamera getOrthographicCamera()
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

   public void addSpriteRenderableProvider(Consumer<ArrayList<RDX2DSpriteDrawable>> spriteRenderableProvider)
   {
      spriteRenderableProviders.add(spriteRenderableProvider);
   }

   public void queueRemoveSpriteRenderableProvider(Consumer<ArrayList<RDX2DSpriteDrawable>> spriteRenderableProvider)
   {
      spriteRenderableRemovalQueue.add(spriteRenderableProvider);
   }
}
