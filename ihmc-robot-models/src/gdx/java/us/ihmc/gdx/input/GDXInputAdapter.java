package us.ihmc.gdx.input;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.InputProcessor;
import imgui.ImGui;
import org.lwjgl.glfw.GLFW;

/**
 * This class provides input for IHMC GDX based apps. There are
 * a few reasons to use this instead of the default GDX way:
 * - In GDX, getDeltaX and getDeltaY seem broken so have to fix drags.
 * - When there's a lot going on in the app, such as ImGui windows
 *   key presses need to be local rather than global.
 */
public class GDXInputAdapter
{
   private static final int GDX_TO_IMGUI_KEY_CODE_OFFSET = GLFW.GLFW_KEY_A - Input.Keys.A;

   PrivateInputProcessor firstInputProcessor;
   InputProcessor secondInputProcessor;

   private volatile int lastDragX = 0;
   private volatile int lastDragY = 0;

   private GDXInputMode mode;

   public GDXInputAdapter(GDXInputMode mode)
   {
      this(new InputAdapter(), mode);
   }

   public GDXInputAdapter(InputProcessor inputProcessor)
   {
      this(inputProcessor, GDXInputMode.libGDX);
   }

   private GDXInputAdapter(InputProcessor inputProcessor, GDXInputMode mode)
   {
      this.mode = mode;
      firstInputProcessor = new PrivateInputProcessor(this);
      secondInputProcessor = inputProcessor;
   }

   public boolean isButtonPressed(int mouseButton)
   {
      if (mode == GDXInputMode.libGDX)
      {
         return Gdx.input.isButtonPressed(mouseButton);
      }
      else
      {
         return ImGui.getIO().getMouseDown(mouseButton);
      }
   }

   public boolean isKeyPressed(int key)
   {
      if (mode == GDXInputMode.libGDX)
      {
         return Gdx.input.isKeyPressed(key);
      }
      else
      {
         return ImGui.getIO().getKeysDown(key + GDX_TO_IMGUI_KEY_CODE_OFFSET);
      }
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
      private final GDXInputAdapter gdxInputAdapter;

      public PrivateInputProcessor(GDXInputAdapter gdxInputAdapter)
      {
         this.gdxInputAdapter = gdxInputAdapter;
      }

      @Override
      public boolean keyDown(int keycode)
      {
         return gdxInputAdapter.keyDown(keycode);
      }

      @Override
      public boolean keyUp(int keycode)
      {
         return gdxInputAdapter.keyUp(keycode);
      }

      @Override
      public boolean keyTyped(char character)
      {
         return gdxInputAdapter.keyTyped(character);
      }

      @Override
      public boolean touchDown(int screenX, int screenY, int pointer, int button)
      {
         lastDragX = screenX;
         lastDragY = screenY;
         return gdxInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean touchUp(int screenX, int screenY, int pointer, int button)
      {
         return gdxInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean touchDragged(int screenX, int screenY, int pointer)
      {
         int deltaX = screenX - lastDragX;
         int deltaY = screenY - lastDragY;
         lastDragX = screenX;
         lastDragY = screenY;
         return gdxInputAdapter.touchDraggedDelta(deltaX, deltaY);
      }

      @Override
      public boolean mouseMoved(int screenX, int screenY)
      {
         return gdxInputAdapter.mouseMoved(screenX, screenY);
      }

      @Override
      public boolean scrolled(float amountX, float amountY)
      {
         return gdxInputAdapter.scrolled(amountX, amountY);
      }
   }

   public InputProcessor getFirstInputProcessor()
   {
      return firstInputProcessor;
   }

   public InputProcessor getSecondInputProcessor()
   {
      return secondInputProcessor;
   }


}
