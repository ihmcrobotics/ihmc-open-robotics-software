package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputAdapter;

import java.util.function.BiConsumer;

/**
 * getDeltaX and getDeltaY are broken so have to track drags.
 *
 * Return boolean is apparently not considered so leave it out of accepted functions.
 */
public class GDXFunctionalInputAdapter extends InputAdapter
{
   private BiConsumer<Float, Float> scrolled;
   private TouchDraggedConsumer touchDragged;

   private volatile boolean dragging = false;
   private volatile int lastDragX = 0;
   private volatile int lastDragY = 0;

   public GDXFunctionalInputAdapter()
   {
   }

   @FunctionalInterface
   public interface TouchDraggedConsumer
   {
      void touchDragged(int deltaX, int deltaY);
   }

   public void setScrolled(BiConsumer<Float, Float> scrolled)
   {
      this.scrolled = scrolled;
   }

   public void setTouchDragged(TouchDraggedConsumer touchDragged)
   {
      this.touchDragged = touchDragged;
   }

   @Override
   public boolean touchDown(int screenX, int screenY, int pointer, int button)
   {
      dragging = true;
      lastDragX = screenX;
      lastDragY = screenY;
      return false;
   }

   @Override
   public boolean touchUp(int screenX, int screenY, int pointer, int button)
   {
      dragging = false;
      return false;
   }

   @Override
   public boolean touchDragged(int screenX, int screenY, int pointer)
   {
      int deltaX = screenX - lastDragX;
      int deltaY = screenY - lastDragY;
      lastDragX = screenX;
      lastDragY = screenY;
      if (touchDragged != null)
      {
         touchDragged.touchDragged(deltaX, deltaY);
      }
      return false;
   }

   @Override
   public boolean mouseMoved(int screenX, int screenY)
   {
      return false;
   }

   @Override
   public boolean scrolled(float amountX, float amountY)
   {
      if (scrolled != null)
      {
         scrolled.accept(amountX, amountY);
      }
      return false;
   }

   @Override
   public boolean keyDown(int keycode)
   {
      return false;
   }

   @Override
   public boolean keyUp(int keycode)
   {
      return false;
   }

   @Override
   public boolean keyTyped(char character)
   {
      return false;
   }
}
