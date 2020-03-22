package us.ihmc.atlas.behaviors.jmeSensorSimulation;

import com.jme3.input.InputManager;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class FocusBasedJMECamera extends Camera
{
   private final Point3D focusPoint;
   private final Quaternion orientation;
   private final Vector3D offsetFromFocusPoint;

   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final Point3D translation;

   private final Vector3f translationJME = new Vector3f();
   private final com.jme3.math.Quaternion orientationJME = new com.jme3.math.Quaternion();

   private boolean leftMousePressed = false;

   public FocusBasedJMECamera(int width, int height, InputManager inputManager)
   {
      super(width, height);

      focusPoint = new Point3D();
      translation = new Point3D(-2.0, 0.7, 1.0);
      offsetFromFocusPoint = new Vector3D(0.0, 0.0, 10.0);
      orientation = new Quaternion();

//      focusPoint = new Vector3f(0.0f, 0.0f, 0.0f);
//      translation = new Vector3f(-2.0f, 0.7f, 1.0f);

      setFrustumPerspective(45.0f, (float) width / height, 1.0f, 1000.0f);

      updateCameraPose();

//      setLocation(translation);
//      lookAt(focusPoint, Vector3f.UNIT_Y);

      JMEInputMapperHelper inputMapper = new JMEInputMapperHelper(inputManager);
      inputMapper.addAnalogMapping("onMouseYUp", new MouseAxisTrigger(MouseInput.AXIS_Y, false), this::onMouseYUp);
      inputMapper.addAnalogMapping("onMouseYDown", new MouseAxisTrigger(MouseInput.AXIS_Y, true), this::onMouseYDown);
      inputMapper.addAnalogMapping("onMouseXLeft", new MouseAxisTrigger(MouseInput.AXIS_X, true), this::onMouseXLeft);
      inputMapper.addAnalogMapping("onMouseXRight", new MouseAxisTrigger(MouseInput.AXIS_X, false), this::onMouseXRight);
      inputMapper.addAnalogMapping("onMouseScrollUp", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false), this::onMouseScrollUp);
      inputMapper.addAnalogMapping("onMouseScrollDown", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true), this::onMouseScrollDown);
      inputMapper.addActionMapping("onMouseButtonLeft", new MouseButtonTrigger(MouseInput.BUTTON_LEFT), this::onMouseButtonLeft);
      inputMapper.addActionMapping("onMouseButtonRight", new MouseButtonTrigger(MouseInput.BUTTON_RIGHT), this::onMouseButtonRight);
      inputMapper.addActionMapping("onKeyW", new KeyTrigger(KeyInput.KEY_W), this::onKeyW);
      inputMapper.addActionMapping("onKeyA", new KeyTrigger(KeyInput.KEY_A), this::onKeyA);
      inputMapper.addActionMapping("onKeyS", new KeyTrigger(KeyInput.KEY_S), this::onKeyS);
      inputMapper.addActionMapping("onKeyD", new KeyTrigger(KeyInput.KEY_D), this::onKeyD);
      inputMapper.addActionMapping("onKeyQ", new KeyTrigger(KeyInput.KEY_Q), this::onKeyQ);
      inputMapper.addActionMapping("onKeyZ", new KeyTrigger(KeyInput.KEY_Z), this::onKeyZ);
      inputMapper.build();
   }

   private void updateCameraPose()
   {
      transform.setToZero();
      transform.appendTranslation(focusPoint);
      transform.setRotation(orientation);
      transform.appendTranslation(offsetFromFocusPoint);

      transform.getTranslation(translation);
      transform.getRotation(orientation);

      translationJME.set(translation.getX32(), translation.getY32(), translation.getZ32());
      orientationJME.set(orientation.getX32(), orientation.getY32(), orientation.getZ32(), orientation.getS32());

      setLocation(translationJME);
      setRotation(orientationJME);
   }

   private void adjustTranslation(float dx, float dy, float dz)
   {

   }

   private void onMouseYUp(float value, float tpf)
   {
      if (leftMousePressed)
      {
         focusPoint.addZ(value);
         updateCameraPose();
      }
   }

   private void onMouseYDown(float value, float tpf)
   {
      if (leftMousePressed)
      {
         focusPoint.subZ(value);
         updateCameraPose();
      }
   }

   private void onMouseXLeft(float value, float tpf)
   {

   }

   private void onMouseXRight(float value, float tpf)
   {

   }

   private void onMouseScrollUp(float value, float tpf)
   {

   }

   private void onMouseScrollDown(float value, float tpf)
   {

   }

   private void onMouseButtonLeft(boolean isPressed, float tpf)
   {
      System.out.println("IS PRESSED " + isPressed);
      leftMousePressed = isPressed;
   }

   private void onMouseButtonRight(boolean isPressed, float tpf)
   {

   }

   private void onKeyW(boolean isPressed, float tpf)
   {
      focusPoint.addX(tpf);
      updateCameraPose();
   }

   private void onKeyA(boolean isPressed, float tpf)
   {

      focusPoint.addY(tpf);
      updateCameraPose();
   }

   private void onKeyS(boolean isPressed, float tpf)
   {
      focusPoint.subX(tpf);
      updateCameraPose();
   }

   private void onKeyD(boolean isPressed, float tpf)
   {
      focusPoint.subY(tpf);
      updateCameraPose();
   }

   private void onKeyQ(boolean isPressed, float tpf)
   {

      updateCameraPose();
   }

   private void onKeyZ(boolean isPressed, float tpf)
   {

      updateCameraPose();
   }
}
