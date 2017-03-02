package us.ihmc.jMonkeyEngineToolkit.jme.input;

import com.google.common.collect.BiMap;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.input.InputManager;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.math.Ray;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Node;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.input.SelectedListenerHolder;
import us.ihmc.jMonkeyEngineToolkit.jme.JMECamera;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.KeyListenerHolder;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyHolder;
import us.ihmc.tools.inputDevices.mouse.MouseButton;
import us.ihmc.tools.inputDevices.mouse.MouseListenerHolder;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DJoystick;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DListener;
import us.ihmc.tools.inputDevices.mouse3DJoystick.Mouse3DListenerHolder;

public class JMEInputManager implements AnalogListener, ActionListener, Mouse3DListener
{
   private final static float mouseFactor = 10.0f;

   private final SelectedListenerHolder selectedListenerHolder;
   private final KeyListenerHolder keyListenerHolder;
   private final MouseListenerHolder mouseListenerHolder;
   private final Mouse3DListenerHolder mouse3DListenerHolder;
   private final Mouse3DJoystick mouse3DJoystick;
   private final BiMap<JMEGraphics3DNode, Graphics3DNode> jmeGraphicsNodes;
   private final Node rootNode;
   private final JMECamera jmeCamera;
   private final InputManager inputManager;
   private boolean reverseY = true;
   private final Object controllerConch = new Object();

   private final ModifierKeyHolder modifierKeyHolder = new ModifierKeyHolder();

   private boolean leftMouseClicked = false;
   private boolean middleMouseClicked = false;
   private boolean rightMouseClicked = false;

   public JMEInputManager(JMERenderer jmeRenderer, Node rootNode, JMECamera jmeCamera, boolean reverseY)
   {
      this.jmeGraphicsNodes = jmeRenderer.getJmeGraphicsNodes().inverse();
      this.jmeCamera = jmeCamera;
      this.rootNode = rootNode;
      this.inputManager = jmeRenderer.getInputManager();
      this.selectedListenerHolder = jmeRenderer.getSelectedListenerHolder();
      this.keyListenerHolder = jmeRenderer.getKeyListenerHolder();
      this.mouseListenerHolder = jmeRenderer.getMouseListenerHolder();
      this.mouse3DListenerHolder = jmeRenderer.getMouse3DListenerHolder();
      this.mouse3DJoystick = jmeRenderer.getMouse3DJoystick();
      this.reverseY = reverseY;
   }

   @Override
   public void onAction(String name, boolean isPressed, float tpf)
   {
      synchronized (controllerConch)
      {
         try
         {
            if (name.equals("LeftMouseClick"))
            {
               leftMouseClicked = isPressed;
            }
            else if (name.equals("MiddleMouseClick"))
            {
               middleMouseClicked = isPressed;
            }
            else if (name.equals("RightMouseClick"))
            {
               rightMouseClicked = isPressed;
            }

            if (name.equals("LeftMouseClick") && !isPressed)
            {
               CollisionResults results = new CollisionResults();

               Vector2f click2d = inputManager.getCursorPosition();
               //JJC 140417 reversed y to fix camera click locations. jme canvas y is reversed 
               if(reverseY) 
               click2d.y = jmeCamera.getHeight()-click2d.y;
               Vector3f click3d = jmeCamera.getWorldCoordinates(
                   new Vector2f(click2d.x, click2d.y), 0f).clone();
               Vector3f direction = jmeCamera.getWorldCoordinates(
                   new Vector2f(click2d.x, click2d.y), 1f).subtractLocal(click3d).normalizeLocal();
               Ray ray = new Ray(click3d, direction);

               rootNode.collideWith(ray, results);

               if (results.size() > 0)
               {
                  CollisionResult closest = results.getClosestCollision();
                  if (closest.getGeometry().getQueueBucket() == Bucket.Sky)
                  {
                     return;
                  }
                  Node parentNode = closest.getGeometry().getParent();

                  while (!jmeGraphicsNodes.containsKey(parentNode))
                  {
                     parentNode = parentNode.getParent();
                     if (parentNode == null)
                     {
                        break;
                     }
                  }

                  Vector3f location3f = new Vector3f(closest.getContactPoint());
                  JMEGeometryUtils.transformFromJMECoordinatesToZup(location3f);

                  Point3D location = JMEDataTypeUtils.jmeVector3fToJ3DPoint3d(location3f);
                  Point3D cameraLocation = jmeCamera.getCameraPosition();
                  QuaternionBasics cameraRotation = jmeCamera.getCameraRotation();

                  Graphics3DNode graphics3dNode = null;
                  if (parentNode != null)
                  {
                     graphics3dNode = jmeGraphicsNodes.get(parentNode);
                     graphics3dNode.notifySelectedListeners(modifierKeyHolder, location, cameraLocation, cameraRotation);
                  }
                  selectedListenerHolder.selected(graphics3dNode, modifierKeyHolder, location, cameraLocation, cameraRotation);
               }

            }
            else if (!name.equals("SelectedObject"))
            {
               final Key key = Key.fromString(name);
               if (isPressed)
               {
                  keyListenerHolder.keyPressed(key);
               }
               else
               {
                  keyListenerHolder.keyReleased(key);
               }
               modifierKeyHolder.setKeyState(key, isPressed);
            }
         }
         catch (Exception e)
         {
            System.err.println("TODO: Fix exception in JMESelectedListener properly");
         }
      }
   }

   @Override
   public void mouseDragged(double dx, double dy, double dz, double drx, double dry, double drz)
   {
      synchronized (controllerConch)
      {
         mouse3DListenerHolder.mouseDragged(dx, dy, dz, drx, dry, drz);
      }
   }

   @Override
   public void onAnalog(String name, float value, float tpf)
   {
      synchronized (controllerConch)
      {
         if (!leftMouseClicked && !middleMouseClicked && !rightMouseClicked)
            return;

         float dx = 0, dy = 0;
         if (name.equals("MouseLeft"))
         {
            dx = -value * mouseFactor;
         }
         else if (name.equals("MouseRight"))
         {
            dx = value * mouseFactor;
         }
         else if (name.equals("MouseUp"))
         {
            dy = value * mouseFactor;
         }
         else if (name.equals("MouseDown"))
         {
            dy = -value * mouseFactor;
         }
         if (leftMouseClicked && rightMouseClicked)
         {
            mouseListenerHolder.mouseDragged(MouseButton.LEFTRIGHT, dx, dy);
         }
         else if (leftMouseClicked)
         {
            mouseListenerHolder.mouseDragged(MouseButton.LEFT, dx, dy);
         }
         else if (middleMouseClicked)
         {
            mouseListenerHolder.mouseDragged(MouseButton.MIDDLE, dx, dy);
         }
         else if (rightMouseClicked)
         {
            mouseListenerHolder.mouseDragged(MouseButton.RIGHT, dx, dy);
         }
      }
   }

   public void registerWithInputManager()
   {
      for (Key modifierKey : Key.values())
      {
         if (modifierKey == Key.UNDEFINED)
            continue;
         inputManager.addMapping(modifierKey.toString(), new KeyTrigger(JMEModifierKey.fromModifierKey(modifierKey)));
         inputManager.addListener(this, modifierKey.toString());
      }

      inputManager.addMapping("LeftMouseClick", new MouseButtonTrigger(MouseInput.BUTTON_LEFT));
      inputManager.addMapping("MiddleMouseClick", new MouseButtonTrigger(MouseInput.BUTTON_MIDDLE));
      inputManager.addMapping("RightMouseClick", new MouseButtonTrigger(MouseInput.BUTTON_RIGHT));

      inputManager.addMapping("MouseRight", new MouseAxisTrigger(MouseInput.AXIS_X, false));
      inputManager.addMapping("MouseLeft", new MouseAxisTrigger(MouseInput.AXIS_X, true));
      inputManager.addMapping("MouseUp", new MouseAxisTrigger(MouseInput.AXIS_Y, true));
      inputManager.addMapping("MouseDown", new MouseAxisTrigger(MouseInput.AXIS_Y, false));

      inputManager.addListener(this, new String[] { "MouseLeft", "MouseRight", "MouseUp", "MouseDown" });
      inputManager.addListener(this, new String[] { "LeftMouseClick", "MiddleMouseClick", "RightMouseClick" });

      mouse3DJoystick.addMouse3DListener(this);
   }

   public void reset()
   {
      leftMouseClicked = false;
      middleMouseClicked = false;
      rightMouseClicked = false;
      for (Key key : Key.values())
      {
         if (modifierKeyHolder.isKeyPressed(key))
         {
            keyListenerHolder.keyReleased(key);
            modifierKeyHolder.setKeyState(key, false);
         }
      }
   }

}
