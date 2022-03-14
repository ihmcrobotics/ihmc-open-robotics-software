package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.*;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.commons.MathTools;
import us.ihmc.gdx.input.ImGui2DViewInput;

public class GDX2DOrthographicCamera extends OrthographicCamera
{
   private final double zoomSpeedFactor = 0.1;
   private final double mouseTranslateSpeedFactor = 2.0;
   private final double keyboardTranslateSpeedFactor = 1000.0;
   private final double maxTranslation = 5000.0;

   private boolean libGDXInputMode = false;
   private boolean isWPressed = false;
   private boolean isAPressed = false;
   private boolean isSPressed = false;
   private boolean isDPressed = false;

   public GDX2DOrthographicCamera()
   {
      viewportWidth = Gdx.graphics.getWidth();
      viewportHeight = Gdx.graphics.getHeight();

      update(true);
   }

   public InputProcessor setInputForLibGDX()
   {
      libGDXInputMode = true;
      return new InputAdapter()
      {
         int lastDragX = 0;
         int lastDragY = 0;

         @Override
         public boolean scrolled(float amountX, float amountY)
         {
            GDX2DOrthographicCamera.this.scrolled(amountY);
            return false;
         }

         @Override
         public boolean touchDown(int screenX, int screenY, int pointer, int button)
         {
            lastDragX = screenX;
            lastDragY = screenY;
            return false;
         }

         @Override
         public boolean touchDragged(int screenX, int screenY, int pointer)
         {
            int deltaX = screenX - lastDragX;
            int deltaY = screenY - lastDragY;
            lastDragX = screenX;
            lastDragY = screenY;
            if (Gdx.input.isButtonPressed(Input.Buttons.LEFT))
            {
               GDX2DOrthographicCamera.this.mouseDragged(deltaX, deltaY);
            }
            return false;
         }
      };
   }

   public void processImGuiInput(ImGui2DViewInput input)
   {
      isWPressed = input.isWindowHovered() && ImGui.isKeyDown('W');
      isSPressed = input.isWindowHovered() && ImGui.isKeyDown('S');
      isAPressed = input.isWindowHovered() && ImGui.isKeyDown('A');
      isDPressed = input.isWindowHovered() && ImGui.isKeyDown('D');

      if (input.isDragging(ImGuiMouseButton.Left))
      {
         mouseDragged(input.getMouseDraggedX(ImGuiMouseButton.Left), input.getMouseDraggedY(ImGuiMouseButton.Left));
      }

      if (input.isWindowHovered() && !ImGui.getIO().getKeyCtrl())
      {
         scrolled(input.getMouseWheelDelta());
      }
   }

   private void mouseDragged(float deltaX, float deltaY)
   {
      translate((float) -mouseTranslateSpeedFactor * zoom * deltaX, (float) mouseTranslateSpeedFactor * zoom * deltaY);
   }

   private void scrolled(float amountY)
   {
      zoom = (float) (zoom + Math.signum(amountY) * zoom * zoomSpeedFactor);
      zoom = (float) MathTools.clamp(zoom, 0.1, 10.0);
   }

   // Taken from GDX PerspectiveCamera

   @Override
   public void update()
   {
      float tpf = Gdx.app.getGraphics().getDeltaTime();

      if (libGDXInputMode)
      {
         isWPressed = Gdx.input.isKeyPressed(Input.Keys.W);
         isSPressed = Gdx.input.isKeyPressed(Input.Keys.S);
         isAPressed = Gdx.input.isKeyPressed(Input.Keys.A);
         isDPressed = Gdx.input.isKeyPressed(Input.Keys.D);
      }

      if (isWPressed)
      {
         translate(0.0f, (float) getKeyboardTranslateSpeedFactor() * tpf);
      }
      if (isSPressed)
      {
         translate(0.0f, (float) -getKeyboardTranslateSpeedFactor() * tpf);
      }
      if (isAPressed)
      {
         translate((float) -getKeyboardTranslateSpeedFactor() * tpf, 0.0f);
      }
      if (isDPressed)
      {
         translate((float) getKeyboardTranslateSpeedFactor() * tpf, 0.0f);
      }

      position.x = (float) MathTools.clamp(position.x, maxTranslation);
      position.y = (float) MathTools.clamp(position.y, maxTranslation);

      update(true);
   }

   private double getKeyboardTranslateSpeedFactor()
   {
      return keyboardTranslateSpeedFactor * zoom;
   }
}