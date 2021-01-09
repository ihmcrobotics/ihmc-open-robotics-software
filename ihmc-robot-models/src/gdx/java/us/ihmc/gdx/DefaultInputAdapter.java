package us.ihmc.gdx;

import com.badlogic.gdx.InputProcessor;

public interface DefaultInputAdapter extends InputProcessor
{
   @Override
   default boolean keyDown(int keycode)
   {
      return false;
   }

   @Override
   default boolean keyUp(int keycode)
   {
      return false;
   }

   @Override
   default boolean keyTyped(char character)
   {
      return false;
   }

   @Override
   default boolean touchDown(int screenX, int screenY, int pointer, int button)
   {
      return false;
   }

   @Override
   default boolean touchUp(int screenX, int screenY, int pointer, int button)
   {
      return false;
   }

   @Override
   default boolean touchDragged(int screenX, int screenY, int pointer)
   {
      return false;
   }

   @Override
   default boolean mouseMoved(int screenX, int screenY)
   {
      return false;
   }

   @Override
   default boolean scrolled(float amountX, float amountY)
   {
      return false;
   }
}
