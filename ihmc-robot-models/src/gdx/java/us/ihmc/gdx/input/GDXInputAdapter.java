package us.ihmc.gdx.input;

import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.InputProcessor;

/**
 * This class provides input for IHMC GDX based apps. There are
 * a few reasons to use this instead of the default GDX way:
 * - In GDX, getDeltaX and getDeltaY seem broken so have to fix drags.
 * - When there's a lot going on in the app, such as ImGui windows
 *   key presses need to be local rather than global.
 */
public class GDXInputAdapter
{
   PrivateInputProcessor firstInputProcessor;
   InputProcessor secondInputProcessor;

   private volatile int lastDragX = 0;
   private volatile int lastDragY = 0;

   public GDXInputAdapter()
   {
      this(new InputAdapter());
   }

   public GDXInputAdapter(InputProcessor inputProcessor)
   {
      firstInputProcessor = new PrivateInputProcessor(this);
      secondInputProcessor = inputProcessor;
   }

   public boolean keyDown(int keycode)
   {
      return secondInputProcessor.keyDown(keycode);
   }

   public boolean keyUp(int keycode)
   {
      return secondInputProcessor.keyUp(keycode);
   }

   public boolean keyTyped(char character)
   {
      return secondInputProcessor.keyTyped(character);
   }

   public boolean touchDown(int screenX, int screenY, int pointer, int button)
   {
      return secondInputProcessor.touchDown(screenX, screenY, pointer, button);
   }

   public boolean touchUp(int screenX, int screenY, int pointer, int button)
   {
      return secondInputProcessor.touchUp(screenX, screenY, pointer, button);
   }

   public boolean touchDraggedDelta(int deltaX, int deltaY)
   {
      return false;
   }

   public boolean mouseMoved(int screenX, int screenY)
   {
      return secondInputProcessor.mouseMoved(screenX, screenY);
   }

   public boolean scrolled(float amountX, float amountY)
   {
      return secondInputProcessor.scrolled(amountX, amountY);
   }

   class PrivateInputProcessor implements InputProcessor
   {
      private final GDXInputAdapter GDXInputAdapter;

      public PrivateInputProcessor(GDXInputAdapter GDXInputAdapter)
      {
         this.GDXInputAdapter = GDXInputAdapter;
      }

      @Override
      public boolean keyDown(int keycode)
      {
         return GDXInputAdapter.keyDown(keycode);
      }

      @Override
      public boolean keyUp(int keycode)
      {
         return GDXInputAdapter.keyUp(keycode);
      }

      @Override
      public boolean keyTyped(char character)
      {
         return GDXInputAdapter.keyTyped(character);
      }

      @Override
      public boolean touchDown(int screenX, int screenY, int pointer, int button)
      {
         lastDragX = screenX;
         lastDragY = screenY;
         return GDXInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean touchUp(int screenX, int screenY, int pointer, int button)
      {
         return GDXInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean touchDragged(int screenX, int screenY, int pointer)
      {
         int deltaX = screenX - lastDragX;
         int deltaY = screenY - lastDragY;
         lastDragX = screenX;
         lastDragY = screenY;
         return GDXInputAdapter.touchDraggedDelta(deltaX, deltaY);
      }

      @Override
      public boolean mouseMoved(int screenX, int screenY)
      {
         return GDXInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean scrolled(float amountX, float amountY)
      {
         return GDXInputAdapter.scrolled(amountX, amountY);
      }
   }

   public InputProcessor getInputProcessor()
   {
      return firstInputProcessor;
   }
}
