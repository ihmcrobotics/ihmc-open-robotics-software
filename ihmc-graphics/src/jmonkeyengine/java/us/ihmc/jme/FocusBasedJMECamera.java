package us.ihmc.jme;

import com.jme3.asset.AssetManager;
import com.jme3.input.InputManager;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.Geometry;
import javafx.scene.paint.Color;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class FocusBasedJMECamera extends Camera
{
   private final PoseReferenceFrame zUpFrame = new PoseReferenceFrame("ZUpFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D cameraPose = new FramePose3D();

   private final AxisAngle latitudeAxisAngle = new AxisAngle();
   private final AxisAngle longitudeAxisAngle = new AxisAngle();
   private final AxisAngle rollAxisAngle = new AxisAngle();

   private final RotationMatrix cameraOrientationOffset = new RotationMatrix();

   private double zoomSpeedFactor = 0.1;
   private double latitudeSpeed = 5.0;
   private double longitudeSpeed = 5.0;
   private double translateSpeed = 5.0;

   private final FramePose3D focusPointPose = new FramePose3D();
   private double latitude = 0.0;
   private double longitude = 0.0;
   private double roll;
   private double zoom = 10.0;

   private final Geometry focusPointSphere;

   private final Vector3D up;
   private final Vector3D forward;
   private final Vector3D left;
   private final Vector3D down;

   private final Vector3f translationJME = new Vector3f();
   private final com.jme3.math.Quaternion orientationJME = new com.jme3.math.Quaternion();

   private boolean leftMousePressed = false;
   private boolean isWPressed = false;
   private boolean isAPressed = false;
   private boolean isSPressed = false;
   private boolean isDPressed = false;
   private boolean isQPressed = false;
   private boolean isZPressed = false;

   public FocusBasedJMECamera(int width, int height, InputManager inputManager, AssetManager assetManager)
   {
      super(width, height);

      setFrustumPerspective(45.0f, (float) width / height, 0.05f, 2000.0f);

      RotationMatrix zUpToYUp = new RotationMatrix();
      zUpToYUp.set(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0);
      zUpFrame.setOrientationAndUpdate(zUpToYUp);

      up = new Vector3D(0.0, 0.0, 1.0);
      forward = new Vector3D(1.0, 0.0, 0.0);
      left = new Vector3D();
      left.cross(up, forward);
      down = new Vector3D();
      down.setAndNegate(up);
      Vector3D cameraZAxis = new Vector3D(forward);
      Vector3D cameraYAxis = new Vector3D(up);
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(cameraYAxis, cameraZAxis);
      cameraOrientationOffset.setColumns(cameraXAxis, cameraYAxis, cameraZAxis);

      JMEMultiColorMeshBuilder colorMeshBuilder = new JMEMultiColorMeshBuilder();
      colorMeshBuilder.addSphere((float) 1.0, new Point3D(0.0, 0.0, 0.0), Color.DARKRED);
      focusPointSphere = new Geometry("FocusPointViz", colorMeshBuilder.generateMesh());
      focusPointSphere.setMaterial(colorMeshBuilder.generateMaterial(assetManager));

      focusPointPose.changeFrame(zUpFrame);
      changeCameraPosition(-2.0, 0.7, 1.0);

      updateCameraPose();

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

   public Geometry getFocusPointSphere()
   {
      return focusPointSphere;
   }

   public void changeCameraPosition(double x, double y, double z)
   {
      Point3D desiredCameraPosition = new Point3D(x, y, z);

      zoom = desiredCameraPosition.distance(focusPointPose.getPosition());

      Vector3D fromFocusToCamera = new Vector3D();
      fromFocusToCamera.sub(desiredCameraPosition, focusPointPose.getPosition());
      fromFocusToCamera.normalize();
      Vector3D fromCameraToFocus = new Vector3D();
      fromCameraToFocus.setAndNegate(fromFocusToCamera);
      // We remove the component along up to be able to compute the longitude
      fromCameraToFocus.scaleAdd(-fromCameraToFocus.dot(down), down, fromCameraToFocus);

      latitude = Math.PI / 2.0 - fromFocusToCamera.angle(down);
      longitude = fromCameraToFocus.angle(forward);

      Vector3D cross = new Vector3D();
      cross.cross(fromCameraToFocus, forward);

      if (cross.dot(down) > 0.0)
         longitude = -longitude;
   }

   private void updateCameraPose()
   {
      zoom = MathTools.clamp(zoom, 0.1, 100.0);

      latitude = MathTools.clamp(latitude, Math.PI / 2.0);
      longitude = EuclidCoreTools.trimAngleMinusPiToPi(longitude);
      roll = 0.0;

      latitudeAxisAngle.set(Axis3D.X, -latitude);
      longitudeAxisAngle.set(Axis3D.Y, -longitude);
      rollAxisAngle.set(Axis3D.Z, roll);

      focusPointPose.changeFrame(ReferenceFrame.getWorldFrame());
      focusPointPose.setOrientation(longitudeAxisAngle);
      focusPointPose.changeFrame(zUpFrame);

      focusPointSphere.setLocalTranslation((float) focusPointPose.getX(), (float) focusPointPose.getY(), (float) focusPointPose.getZ());
      focusPointSphere.setLocalScale((float) (0.0035 * zoom));

      cameraPose.setToZero(zUpFrame);
      cameraPose.appendTranslation(focusPointPose.getPosition());
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
      cameraPose.appendRotation(cameraOrientationOffset);
      cameraPose.appendRotation(longitudeAxisAngle);
      cameraPose.appendRotation(latitudeAxisAngle);
      cameraPose.appendRotation(rollAxisAngle);
      cameraPose.appendTranslation(0.0, 0.0, -zoom);

      translationJME.set(cameraPose.getPosition().getX32(), cameraPose.getPosition().getY32(), cameraPose.getPosition().getZ32());
      orientationJME.set(cameraPose.getOrientation().getX32(),
                         cameraPose.getOrientation().getY32(),
                         cameraPose.getOrientation().getZ32(),
                         cameraPose.getOrientation().getS32());

      setFrame(translationJME, orientationJME);
   }

   public void simpleUpdate(float tpf)
   {
      if (isWPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, translateSpeed * tpf);
      }
      if (isAPressed)
      {
         focusPointPose.appendTranslation(translateSpeed * tpf, 0.0, 0.0);
      }
      if (isSPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, -translateSpeed * tpf);
      }
      if (isDPressed)
      {
         focusPointPose.appendTranslation(-translateSpeed * tpf, 0.0, 0.0);
      }
      if (isQPressed)
      {
         focusPointPose.appendTranslation(0.0, translateSpeed * tpf, 0.0);
      }
      if (isZPressed)
      {
         focusPointPose.appendTranslation(0.0, -translateSpeed * tpf, 0.0);
      }

      updateCameraPose();
   }

   private void onMouseYUp(float value, float tpf)
   {
      if (leftMousePressed)
      {
         latitude += latitudeSpeed * value;
      }
   }

   private void onMouseYDown(float value, float tpf)
   {
      if (leftMousePressed)
      {
         latitude -= latitudeSpeed * value;
      }
   }

   private void onMouseXLeft(float value, float tpf)
   {
      if (leftMousePressed)
      {
         longitude -= longitudeSpeed * value;
      }
   }

   private void onMouseXRight(float value, float tpf)
   {
      if (leftMousePressed)
      {
         longitude += longitudeSpeed * value;
      }
   }

   private void onMouseScrollUp(float value, float tpf)
   {
      zoom = zoom - zoom * zoomSpeedFactor;
//      zoom -= zoomSpeedFactor * value;
   }

   private void onMouseScrollDown(float value, float tpf)
   {
      zoom = zoom + zoom * zoomSpeedFactor;
//      zoom += zoomSpeedFactor * value;
   }

   private void onMouseButtonLeft(boolean isPressed, float tpf)
   {
      leftMousePressed = isPressed;
   }

   private void onMouseButtonRight(boolean isPressed, float tpf)
   {

   }

   private void onKeyW(boolean isPressed, float tpf)
   {
      isWPressed = isPressed;
   }

   private void onKeyA(boolean isPressed, float tpf)
   {
      isAPressed = isPressed;
   }

   private void onKeyS(boolean isPressed, float tpf)
   {
      isSPressed = isPressed;
   }

   private void onKeyD(boolean isPressed, float tpf)
   {
      isDPressed = isPressed;
   }

   private void onKeyQ(boolean isPressed, float tpf)
   {
      isQPressed = isPressed;
   }

   private void onKeyZ(boolean isPressed, float tpf)
   {
      isZPressed = isPressed;
   }
}