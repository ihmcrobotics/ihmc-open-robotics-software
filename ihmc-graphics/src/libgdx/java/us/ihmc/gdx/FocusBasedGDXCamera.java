package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import imgui.internal.ImGui;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;

public class FocusBasedGDXCamera extends Camera
{
   private final FramePose3D cameraPose = new FramePose3D();

   private final Vector3D euclidDirection = new Vector3D();
   private final Vector3D euclidUp = new Vector3D();

   private final AxisAngle latitudeAxisAngle = new AxisAngle();
   private final AxisAngle longitudeAxisAngle = new AxisAngle();
   private final AxisAngle rollAxisAngle = new AxisAngle();
   private final AxisAngle focusPointAxisAngle = new AxisAngle();

   private final RotationMatrix cameraOrientationOffset = new RotationMatrix();

   private float fieldOfView;

   private double zoomSpeedFactor = 0.1;
   private double latitudeSpeed = 0.005;
   private double longitudeSpeed = 0.005;
   private double translateSpeedFactor = 0.5;

   private final FramePose3D focusPointPose = new FramePose3D();
   private double latitude = 0.0;
   private double longitude = 0.0;
   private double roll;
   private double zoom = 10.0;

   private final Model focusPointModel;
   private final ModelInstance focusPointSphere;

   private final Vector3D cameraOffsetUp;
   private final Vector3D cameraOffsetForward;
   private final Vector3D cameraOffsetLeft;
   private final Vector3D cameraOffsetDown;

   private boolean libGDXInputMode = false;
   private boolean isWPressed = false;
   private boolean isAPressed = false;
   private boolean isSPressed = false;
   private boolean isDPressed = false;
   private boolean isQPressed = false;
   private boolean isZPressed = false;

   public FocusBasedGDXCamera()
   {
      fieldOfView = 45.0f;
      viewportWidth = Gdx.graphics.getWidth();
      viewportHeight = Gdx.graphics.getHeight();
      near = 0.05f;
      far = 2000.0f;

      cameraOffsetUp = new Vector3D(0.0, 0.0, 1.0);
      cameraOffsetForward = new Vector3D(1.0, 0.0, 0.0);
      cameraOffsetLeft = new Vector3D();
      cameraOffsetLeft.cross(cameraOffsetUp, cameraOffsetForward);
      cameraOffsetDown = new Vector3D();
      cameraOffsetDown.setAndNegate(cameraOffsetUp);
      Vector3D cameraZAxis = new Vector3D(cameraOffsetForward);
      Vector3D cameraYAxis = new Vector3D(cameraOffsetUp);
      Vector3D cameraXAxis = new Vector3D();
      cameraXAxis.cross(cameraYAxis, cameraZAxis);
      cameraOrientationOffset.setColumns(cameraXAxis, cameraYAxis, cameraZAxis);

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();
      modelBuilder.node().id = "focusPointSphere"; // optional
      GDXMultiColorMeshBuilder meshBuilder = new GDXMultiColorMeshBuilder();
      meshBuilder.addSphere((float) 1.0, new Point3D(0.0, 0.0, 0.0), new Color(0.54509807f, 0.0f, 0.0f, 1.0f)); // dark red
      Mesh mesh = meshBuilder.generateMesh();
      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL20.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = new Texture(Gdx.files.classpath("palette.png"));
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(com.badlogic.gdx.graphics.Color.WHITE));
      modelBuilder.part(meshPart, material);
      focusPointModel = modelBuilder.end();
      focusPointSphere = new ModelInstance(focusPointModel);

      changeCameraPosition(-2.0, 0.7, 1.0);

      updateCameraPose();
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
            FocusBasedGDXCamera.this.scrolled(amountY);
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
               FocusBasedGDXCamera.this.mouseDragged(deltaX, deltaY);
            }
            return false;
         }
      };
   }

   public ModelInstance getFocusPointSphere()
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
      fromCameraToFocus.scaleAdd(-fromCameraToFocus.dot(cameraOffsetDown), cameraOffsetDown, fromCameraToFocus);

      latitude = Math.PI / 2.0 - fromFocusToCamera.angle(cameraOffsetDown);
      longitude = fromCameraToFocus.angle(cameraOffsetForward);

      Vector3D cross = new Vector3D();
      cross.cross(fromCameraToFocus, cameraOffsetForward);

      if (cross.dot(cameraOffsetDown) > 0.0)
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

      focusPointAxisAngle.set(Axis3D.Z, -longitude);

      focusPointPose.changeFrame(ReferenceFrame.getWorldFrame());
      focusPointPose.getOrientation().set(focusPointAxisAngle);

      focusPointSphere.nodes.get(0).translation.set((float) focusPointPose.getX(), (float) focusPointPose.getY(), (float) focusPointPose.getZ());
      focusPointSphere.nodes.get(0).scale.set((float) (0.0035 * zoom), (float) (0.0035 * zoom), (float) (0.0035 * zoom));
      focusPointSphere.calculateTransforms();

      cameraPose.setToZero(ReferenceFrame.getWorldFrame());
      cameraPose.appendTranslation(focusPointPose.getPosition());
      cameraPose.appendRotation(cameraOrientationOffset);
      cameraPose.appendRotation(longitudeAxisAngle);
      cameraPose.appendRotation(latitudeAxisAngle);
      cameraPose.appendRotation(rollAxisAngle);
      cameraPose.appendTranslation(0.0, 0.0, -zoom);

      euclidDirection.set(Axis3D.Z);
      cameraPose.getOrientation().transform(euclidDirection);
      euclidUp.set(Axis3D.Y); // camera is rendered in Y up
      cameraPose.getOrientation().transform(euclidUp);

      position.set(cameraPose.getPosition().getX32(), cameraPose.getPosition().getY32(), cameraPose.getPosition().getZ32());
      direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
      up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());
   }

   public void processImGuiInput(ImGui3DViewInput input)
   {
      isWPressed = input.isWindowHovered() && ImGui.isKeyDown('W');
      isSPressed = input.isWindowHovered() && ImGui.isKeyDown('S');
      isAPressed = input.isWindowHovered() && ImGui.isKeyDown('A');
      isDPressed = input.isWindowHovered() && ImGui.isKeyDown('D');
      isQPressed = input.isWindowHovered() && ImGui.isKeyDown('Q');
      isZPressed = input.isWindowHovered() && ImGui.isKeyDown('Z');

      if (input.isDraggingLeft())
      {
         mouseDragged(input.getMouseDraggedX(), input.getMouseDraggedY());
      }

      if (input.isWindowHovered() && !ImGui.getIO().getKeyCtrl())
      {
         scrolled(input.getMouseWheelDelta());
      }
   }

   private void mouseDragged(float deltaX, float deltaY)
   {
      latitude -= latitudeSpeed * deltaY;
      longitude += longitudeSpeed * deltaX;
   }

   private void scrolled(float amountY)
   {
      zoom = zoom + Math.signum(amountY) * zoom * zoomSpeedFactor;
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
         isQPressed = Gdx.input.isKeyPressed(Input.Keys.Q);
         isZPressed = Gdx.input.isKeyPressed(Input.Keys.Z);
      }

      if (isWPressed)
      {
         focusPointPose.appendTranslation(getTranslateSpeedFactor() * tpf, 0.0, 0.0);
      }
      if (isSPressed)
      {
         focusPointPose.appendTranslation(-getTranslateSpeedFactor() * tpf, 0.0, 0.0);
      }
      if (isAPressed)
      {
         focusPointPose.appendTranslation(0.0, getTranslateSpeedFactor() * tpf, 0.0);
      }
      if (isDPressed)
      {
         focusPointPose.appendTranslation(0.0, -getTranslateSpeedFactor() * tpf, 0.0);
      }
      if (isQPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, getTranslateSpeedFactor() * tpf);
      }
      if (isZPressed)
      {
         focusPointPose.appendTranslation(0.0, 0.0, -getTranslateSpeedFactor() * tpf);
      }

      updateCameraPose();

      update(true);
   }

   final Vector3 tmp = new Vector3();

   @Override
   public void update(boolean updateFrustum)
   {
      float aspect = viewportWidth / viewportHeight;
      projection.setToProjection(Math.abs(near), Math.abs(far), fieldOfView, aspect);
      view.setToLookAt(position, tmp.set(position).add(direction), up);
      combined.set(projection);
      Matrix4.mul(combined.val, view.val);

      if (updateFrustum)
      {
         invProjectionView.set(combined);
         Matrix4.inv(invProjectionView.val);
         frustum.update(invProjectionView);
      }
   }

   public void dispose()
   {
      focusPointModel.dispose();
   }

   private double getTranslateSpeedFactor()
   {
      return translateSpeedFactor * zoom;
   }
}