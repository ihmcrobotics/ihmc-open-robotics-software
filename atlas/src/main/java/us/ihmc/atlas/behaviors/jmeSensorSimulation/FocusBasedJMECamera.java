package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.input.InputManager;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;

public class FocusBasedJMECamera extends Camera implements ActionListener, AnalogListener
{
   private static final String MOUSE_Y_UP = "MouseYUp";
   private static final String MOUSE_Y_DOWN = "MouseYDown";
   private static final String MOUSE_X_LEFT = "MouseXLeft";
   private static final String MOUSE_X_RIGHT = "MouseXRight";
   private static final String MOUSE_SCROLL_UP = "MouseScrollUp";
   private static final String MOUSE_SCROLL_DOWN = "MouseScrollDown";
   private static final String MOUSE_BUTTON_LEFT = "MouseButtonLeft";
   private static final String MOUSE_BUTTON_RIGHT = "MouseButtonRight";

   private final Vector3f focusPoint;
   private final Vector3f translation;

   private boolean leftMousePressed = false;

   public FocusBasedJMECamera(int width, int height, InputManager inputManager)
   {
      super(width, height);

      focusPoint = new Vector3f(0f, 0f, 0f);
      translation = new Vector3f(-2.0f, 0.7f, 1.0f);

      setFrustumPerspective(45f, (float) width / height, 1f, 1000f);
      setLocation(translation);
      lookAt(focusPoint, Vector3f.UNIT_Z);

      JMEInputMapperHelper inputMapper = new JMEInputMapperHelper(inputManager);
      inputMapper.addAnalogMapping("onMouseYUp", new MouseAxisTrigger(MouseInput.AXIS_Y, false), this::onMouseYUp);
      inputMapper.addAnalogMapping("onMouseYDown", new MouseAxisTrigger(MouseInput.AXIS_Y, false), this::onMouseYDown);
      inputManager.addMapping(MOUSE_Y_UP, new MouseAxisTrigger(MouseInput.AXIS_Y, false));
      inputManager.addMapping(MOUSE_Y_DOWN, new MouseAxisTrigger(MouseInput.AXIS_Y, true));
      inputManager.addMapping(MOUSE_X_LEFT, new MouseAxisTrigger(MouseInput.AXIS_X, true));
      inputManager.addMapping(MOUSE_X_RIGHT, new MouseAxisTrigger(MouseInput.AXIS_X, false));
      inputManager.addMapping(MOUSE_SCROLL_UP, new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));
      inputManager.addMapping(MOUSE_SCROLL_DOWN, new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));
      inputManager.addMapping(MOUSE_BUTTON_LEFT, new MouseButtonTrigger(MouseInput.BUTTON_LEFT));
      inputManager.addMapping(MOUSE_BUTTON_RIGHT, new MouseButtonTrigger(MouseInput.BUTTON_RIGHT));

      inputManager.addListener(this,
                               MOUSE_Y_UP,
                               MOUSE_Y_DOWN,
                               MOUSE_X_LEFT,
                               MOUSE_X_RIGHT,
                               MOUSE_SCROLL_UP,
                               MOUSE_SCROLL_DOWN,
                               MOUSE_BUTTON_LEFT,
                               MOUSE_BUTTON_RIGHT);
   }

   private void onMouseYUp(float value, float tpf)
   {

   }

   private void onMouseYDown(float value, float tpf)
   {

   }

   private void onMouseYUp(float value, float tpf)
   {

   }

   private void onMouseYUp(float value, float tpf)
   {

   }

   private void onMouseYUp(float value, float tpf)
   {

   }

   private void onMouseYUp(float value, float tpf)
   {

   }

   private void onMouseYUp(float value, float tpf)
   {

   }

   @Override
   public void onAction(String name, boolean isPressed, float tpf)
   {
      if (name.equals(MOUSE_BUTTON_LEFT))
      {
         leftMousePressed = isPressed;
      }

//      System.out.println(name + " isPressed " + isPressed + " tpf " + tpf);
   }

   @Override
   public void onAnalog(String name, float value, float tpf)
   {
      if (name.equals(MOUSE_Y_UP))
      {

      }
      else if (name.equals(MOUSE_Y_DOWN))
      {

      }
      else if (name.equals(MOUSE_X_LEFT))
      {

      }
      else if (name.equals(MOUSE_X_RIGHT))
      {

      }
      else if (name.equals(MOUSE_SCROLL_UP))
      {

      }
      else if (name.equals(MOUSE_SCROLL_DOWN))
      {

      }
//      System.out.println(name + " value " + value + " tpf " + tpf);
   }
}
