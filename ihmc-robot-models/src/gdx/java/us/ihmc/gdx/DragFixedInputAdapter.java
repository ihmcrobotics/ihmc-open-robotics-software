package us.ihmc.gdx;

import com.badlogic.gdx.InputProcessor;

/**
 * getDeltaX and getDeltaY are broken so have to track drags.
 */
public class DragFixedInputAdapter
{
   PrivateInputProcessor inputProcessor;

   private volatile int lastDragX = 0;
   private volatile int lastDragY = 0;

   public DragFixedInputAdapter()
   {
      inputProcessor = new PrivateInputProcessor(this);
   }

   public boolean keyDown(int keycode)
   {
      return false;
   }

   public boolean keyUp(int keycode)
   {
      return false;
   }

   public boolean keyTyped(char character)
   {
      return false;
   }

   public boolean touchDown(int screenX, int screenY, int pointer, int button)
   {
      return false;
   }

   public boolean touchUp(int screenX, int screenY, int pointer, int button)
   {
      return false;
   }

   public boolean touchDragged(int deltaX, int deltaY)
   {
      return false;
   }

   public boolean mouseMoved(int screenX, int screenY)
   {
      return false;
   }

   public boolean scrolled(float amountX, float amountY)
   {
      return false;
   }

   class PrivateInputProcessor implements InputProcessor
   {
      private final DragFixedInputAdapter dragFixedInputAdapter;

      public PrivateInputProcessor(DragFixedInputAdapter dragFixedInputAdapter)
      {
         this.dragFixedInputAdapter = dragFixedInputAdapter;
      }

      @Override
      public boolean keyDown(int keycode)
      {
         return dragFixedInputAdapter.keyDown(keycode);
      }

      @Override
      public boolean keyUp(int keycode)
      {
         return dragFixedInputAdapter.keyUp(keycode);
      }

      @Override
      public boolean keyTyped(char character)
      {
         return dragFixedInputAdapter.keyTyped(character);
      }

      @Override
      public boolean touchDown(int screenX, int screenY, int pointer, int button)
      {
         lastDragX = screenX;
         lastDragY = screenY;
         return dragFixedInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean touchUp(int screenX, int screenY, int pointer, int button)
      {
         return dragFixedInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean touchDragged(int screenX, int screenY, int pointer)
      {
         int deltaX = screenX - lastDragX;
         int deltaY = screenY - lastDragY;
         lastDragX = screenX;
         lastDragY = screenY;
         return dragFixedInputAdapter.touchDragged(deltaX, deltaY);
      }

      @Override
      public boolean mouseMoved(int screenX, int screenY)
      {
         return dragFixedInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean scrolled(float amountX, float amountY)
      {
         return dragFixedInputAdapter.scrolled(amountX, amountY);
      }
   }

   public InputProcessor getInputProcessor()
   {
      return inputProcessor;
   }
}
