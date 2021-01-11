package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
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
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.ArrayList;

public class FocusBasedGDXCamera extends Camera
{
   private final DragFixedInputAdapter dragFixedInputAdapter;

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
   private double translateSpeed = 5.0;

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

   private int lastSeenMouseX;
   private int lastSeenMouseY;
   private final BoundingBox2D mainBoundary = new BoundingBox2D();
   private final ArrayList<BoundingBox2D> exclusionBoundingBoxes = new ArrayList<>();

   public FocusBasedGDXCamera()
   {
      fieldOfView = 45.0f;
      viewportWidth = Gdx.graphics.getWidth();
      viewportHeight = Gdx.graphics.getHeight();
      near = 0.05f;
      far = 2000.0f;

      mainBoundary.setMax((int) viewportWidth, (int) viewportHeight);

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

      dragFixedInputAdapter = new DragFixedInputAdapter()
      {
         @Override
         public boolean scrolled(float amountX, float amountY)
         {
            return FocusBasedGDXCamera.this.scrolled(amountX, amountY);
         }

         @Override
         public boolean mouseMoved(int screenX, int screenY)
         {
            lastSeenMouseX = screenX;
            lastSeenMouseY = screenY;
            return false;
         }

         @Override
         public boolean touchUp(int screenX, int screenY, int pointer, int button)
         {
            lastSeenMouseX = screenX;
            lastSeenMouseY = screenY;
            return false;
         }

         @Override
         public boolean touchDown(int screenX, int screenY, int pointer, int button)
         {
            lastSeenMouseX = screenX;
            lastSeenMouseY = screenY;
            return false;
         }

         @Override
         public boolean touchDragged(int deltaX, int deltaY)
         {
            return FocusBasedGDXCamera.this.touchDragged(deltaX, deltaY);
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

      euclidDirection.set(0.0, 0.0, 1.0);
      cameraPose.getOrientation().transform(euclidDirection);
      euclidUp.set(0.0, 1.0, 0.0);
      cameraPose.getOrientation().transform(euclidUp);

      position.set(cameraPose.getPosition().getX32(), cameraPose.getPosition().getY32(), cameraPose.getPosition().getZ32());
      direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
      up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());
   }

   public boolean touchDragged(int deltaX, int deltaY)
   {
      if (inputInBounds())
      {
         if (Gdx.input.isButtonPressed(Input.Buttons.LEFT))
         {
            latitude -= latitudeSpeed * deltaY;
            longitude += longitudeSpeed * deltaX;

            return true;
         }
      }
      return false;
   }

   public boolean scrolled(float amountX, float amountY)
   {
      if (inputInBounds())
      {
         zoom = zoom + Math.signum(amountY) * zoom * zoomSpeedFactor;
         return true;
      }

      return false;
   }

   private boolean inputInBounds()
   {
      boolean inBounds = mainBoundary.isInsideInclusive(lastSeenMouseX, lastSeenMouseY);

      for (BoundingBox2D exclusionBoundingBox : exclusionBoundingBoxes)
      {
         inBounds &= !exclusionBoundingBox.isInsideInclusive(lastSeenMouseX, lastSeenMouseY);
      }

      return inBounds;
   }

   // Taken from GDX PerspectiveCamera

   @Override
   public void update()
   {
      float tpf = Gdx.app.getGraphics().getDeltaTime();

      if (Gdx.input.isKeyPressed(Input.Keys.W))
      {
         focusPointPose.appendTranslation(translateSpeed * tpf, 0.0, 0.0);
      }
      if (Gdx.input.isKeyPressed(Input.Keys.S))
      {
         focusPointPose.appendTranslation(-translateSpeed * tpf, 0.0, 0.0);
      }
      if (Gdx.input.isKeyPressed(Input.Keys.A))
      {
         focusPointPose.appendTranslation(0.0, translateSpeed * tpf, 0.0);
      }
      if (Gdx.input.isKeyPressed(Input.Keys.D))
      {
         focusPointPose.appendTranslation(0.0, -translateSpeed * tpf, 0.0);
      }
      if (Gdx.input.isKeyPressed(Input.Keys.Q))
      {
         focusPointPose.appendTranslation(0.0, 0.0, translateSpeed * tpf);
      }
      if (Gdx.input.isKeyPressed(Input.Keys.Z))
      {
         focusPointPose.appendTranslation(0.0, 0.0, -translateSpeed * tpf);
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

   public InputProcessor getInputProcessor()
   {
      return dragFixedInputAdapter.getInputProcessor();
   }

   public void dispose()
   {
      focusPointModel.dispose();
   }

   public void setInputBounds(int minX, int maxX, int minY, int maxY)
   {
      mainBoundary.set(minX, minY, maxX, maxY);
   }

   public void setInputExclusionBoxes(ArrayList<BoundingBox2D> exclusionBoundingBoxes)
   {
      exclusionBoundingBoxes.clear();
      exclusionBoundingBoxes.addAll(exclusionBoundingBoxes);
   }

   public void addInputExclusionBox(BoundingBox2D exclusionBoundingBox)
   {
      exclusionBoundingBoxes.add(exclusionBoundingBox);
   }

   public void clearInputExclusionBoxes()
   {
      exclusionBoundingBoxes.clear();
   }
}