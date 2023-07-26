package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.*;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.commons.MathTools;
import us.ihmc.rdx.input.ImGui2DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDX2DOrthographicCamera extends OrthographicCamera
{
   private double zoomSpeedFactor = 0.1;
   private double mouseTranslateSpeedFactor = 0.02;
   private double keyboardTranslateSpeedFactor = 10.0;
   private double minTranslationX = -100.0;
   private double minTranslationY = -100.0;
   private double maxTranslationX = 100.0;
   private double maxTranslationY = 100.0;
   private double maxZoom = 2.0;
   private double minZoom = 0.1;

   private boolean libGDXInputMode = false;
   private boolean isWPressed = false;
   private boolean isAPressed = false;
   private boolean isSPressed = false;
   private boolean isDPressed = false;

   public RDX2DOrthographicCamera()
   {
      viewportWidth = Gdx.graphics.getWidth();
      viewportHeight = Gdx.graphics.getHeight();

      update(true);

      RDXBaseUI.getInstance().getKeyBindings().register("Move forward", "W");
      RDXBaseUI.getInstance().getKeyBindings().register("Move back", "S");
      RDXBaseUI.getInstance().getKeyBindings().register("Move left", "A");
      RDXBaseUI.getInstance().getKeyBindings().register("Move right", "D");
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
            RDX2DOrthographicCamera.this.scrolled(amountY);
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
               RDX2DOrthographicCamera.this.mouseDragged(deltaX, deltaY);
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
      zoom = (float) MathTools.clamp(zoom, minZoom, maxZoom);
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

      position.x = (float) MathTools.clamp(position.x, minTranslationX, maxTranslationX);
      position.y = (float) MathTools.clamp(position.y, minTranslationY, maxTranslationY);

      update(true);
   }

   private double getKeyboardTranslateSpeedFactor()
   {
      return keyboardTranslateSpeedFactor * zoom;
   }

   public double getMinZoom()
   {
      return minZoom;
   }

   public void setMinZoom(double minZoom)
   {
      this.minZoom = minZoom;
   }

   public double getMaxZoom()
   {
      return maxZoom;
   }

   public void setMaxZoom(double maxZoom)
   {
      this.maxZoom = maxZoom;
   }

   public double getMinTranslationX()
   {
      return minTranslationX;
   }

   public void setMinTranslationX(double minTranslationX)
   {
      this.minTranslationX = minTranslationX;
   }

   public double getMaxTranslationX()
   {
      return maxTranslationX;
   }

   public void setMaxTranslationX(double maxTranslationX)
   {
      this.maxTranslationX = maxTranslationX;
   }

   public double getMinTranslationY()
   {
      return minTranslationY;
   }

   public void setMinTranslationY(double minTranslationY)
   {
      this.minTranslationY = minTranslationY;
   }

   public double getMaxTranslationY()
   {
      return maxTranslationY;
   }

   public void setMaxTranslationY(double maxTranslationY)
   {
      this.maxTranslationY = maxTranslationY;
   }

   public double getZoomSpeedFactor()
   {
      return zoomSpeedFactor;
   }

   public void setZoomSpeedFactor(double zoomSpeedFactor)
   {
      this.zoomSpeedFactor = zoomSpeedFactor;
   }

   public double getMouseTranslateSpeedFactor()
   {
      return mouseTranslateSpeedFactor;
   }

   public void setKeyboardTranslateSpeedFactor(double keyboardTranslateSpeedFactor)
   {
      this.keyboardTranslateSpeedFactor = keyboardTranslateSpeedFactor;
   }
}