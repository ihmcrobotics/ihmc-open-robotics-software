package us.ihmc.gdx.input;

public interface GDXInputDeliveryInterface
{
   boolean isButtonPressed(int mouseButton);

   boolean isKeyPressed(int key);

   void onKeyDown(int keycode);

   void onKeyUp(int keycode);

   void onKeyTyped(char character);

   void onMouseClicked(int screenX, int screenY, int button);

   void onMouseDown(int screenX, int screenY, int button);

   void onMouseUp(int screenX, int screenY, int button);

   void onMouseDraggedDelta(int deltaX, int deltaY, int button);

   void onMouseMoved(int screenX, int screenY);

   void onMouseScrolled(float amountX, float amountY);
}
