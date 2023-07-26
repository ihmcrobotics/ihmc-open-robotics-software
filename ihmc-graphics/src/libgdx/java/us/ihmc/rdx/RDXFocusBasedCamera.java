package us.ihmc.rdx;

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
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGuiMouseDragData;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXFocusBasedCamera extends Camera
{
   private boolean inputEnabled = true;
   private boolean useMiddleClickViewOrbit = false;
   private final FramePose3D cameraPose = new FramePose3D();
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final ReferenceFrame cameraFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     transformToParent);

   private final FrameVector3D euclidDirection = new FrameVector3D();
   private final FrameVector3D euclidUp = new FrameVector3D();

   private final AxisAngle latitudeAxisAngle = new AxisAngle();
   private final AxisAngle focusPointAxisAngle = new AxisAngle();

   private final double zoomSpeedFactor = 0.1;
   private final double latitudeSpeed = 0.005;
   private final double longitudeSpeed = 0.005;
   private final double translateSpeedFactor = 0.5;

   private final FramePose3D focusPointPose;
   private float verticalFieldOfView;
   private double latitude = 0.0;
   private double longitude = 0.0;
   private double zoom = 10.0;

   private final Model focusPointModel;
   private final ModelInstance focusPointSphere;

   private final Vector3D NEGATIVE_Z = new Vector3D(0.0, 0.0, -1.0);

   private boolean libGDXInputMode = false;
   private boolean isWPressed = false;
   private boolean isAPressed = false;
   private boolean isSPressed = false;
   private boolean isDPressed = false;
   private boolean isQPressed = false;
   private boolean isZPressed = false;
   private boolean isCPressed = false;
   private boolean isEPressed = false;

   public RDXFocusBasedCamera()
   {
      focusPointPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      verticalFieldOfView = 45.0f;
      viewportWidth = Gdx.graphics.getWidth();
      viewportHeight = Gdx.graphics.getHeight();
      near = 0.05f;
      far = 2000.0f;

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();
      modelBuilder.node().id = "focusPointSphere"; // optional
      RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
      meshBuilder.addSphere((float) 1.0, new Point3D(0.0, 0.0, 0.0), new Color(0.54509807f, 0.0f, 0.0f, 1.0f)); // dark red
      Mesh mesh = meshBuilder.generateMesh();
      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL20.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(TextureAttribute.createDiffuse(paletteTexture));
      material.set(ColorAttribute.createDiffuse(com.badlogic.gdx.graphics.Color.WHITE));
      modelBuilder.part(meshPart, material);
      focusPointModel = modelBuilder.end();
      focusPointSphere = new ModelInstance(focusPointModel);

      changeCameraPosition(-2.0, 0.7, 1.0);

      updateCameraPose();
      update(true);

      RDXBaseUI.getInstance().getKeyBindings().register("Move camera forward ", "W");
      RDXBaseUI.getInstance().getKeyBindings().register("Move camera back", "S");
      RDXBaseUI.getInstance().getKeyBindings().register("Move camera left", "A");
      RDXBaseUI.getInstance().getKeyBindings().register("Move camera right", "D");
      RDXBaseUI.getInstance().getKeyBindings().register("Move camera up", "Q");
      RDXBaseUI.getInstance().getKeyBindings().register("Move camera down", "Z");
      RDXBaseUI.getInstance().getKeyBindings().register("Zoom camera in (broken?)", "C");
      RDXBaseUI.getInstance().getKeyBindings().register("Zoom camera out (broken?)", "E");
      RDXBaseUI.getInstance().getKeyBindings().register("Zoom camera in / out", "Mouse scroll");
      RDXBaseUI.getInstance().getKeyBindings().register("Fine adjustment", "Shift");
      RDXBaseUI.getInstance().getKeyBindings().register("Drag to orient camera", "Left mouse");
      RDXBaseUI.getInstance().getKeyBindings().register("Drag to pan camera", "Middle mouse");
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
            RDXFocusBasedCamera.this.scrolled(amountY);
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
               RDXFocusBasedCamera.this.mouseDragged(deltaX, deltaY);
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
      fromCameraToFocus.scaleAdd(-fromCameraToFocus.dot(NEGATIVE_Z), NEGATIVE_Z, fromCameraToFocus);

      latitude = Math.PI / 2.0 - fromFocusToCamera.angle(NEGATIVE_Z);
      longitude = fromCameraToFocus.angle(Axis3D.X);

      Vector3D cross = new Vector3D();
      cross.cross(fromCameraToFocus, Axis3D.X);

      if (cross.dot(NEGATIVE_Z) > 0.0)
         longitude = -longitude;
   }

   public void translateCameraFocusPoint(Tuple3DReadOnly translation)
   {
      focusPointPose.getPosition().add(translation);
   }

   public void setCameraFocusPoint(Tuple3DReadOnly translation)
   {
      focusPointPose.getPosition().set(translation);
   }

   public void updateCameraPose()
   {
      zoom = MathTools.clamp(zoom, 0.1, 100.0);

      latitude = MathTools.clamp(latitude, Math.PI / 2.0);
      longitude = EuclidCoreTools.trimAngleMinusPiToPi(longitude);

      focusPointAxisAngle.set(Axis3D.Z, -longitude);
      focusPointPose.getOrientation().set(focusPointAxisAngle);

      LibGDXTools.toLibGDX(focusPointPose.getPosition(), focusPointSphere.nodes.get(0).translation);
      focusPointSphere.nodes.get(0).scale.set((float) (0.0035 * zoom), (float) (0.0035 * zoom), (float) (0.0035 * zoom));
      focusPointSphere.calculateTransforms();

      cameraPose.setIncludingFrame(focusPointPose);
      latitudeAxisAngle.set(Axis3D.Y, -latitude);
      cameraPose.appendRotation(latitudeAxisAngle);
      cameraPose.appendTranslation(-zoom, 0.0, 0.0);

      euclidDirection.setIncludingFrame(ReferenceFrame.getWorldFrame(), Axis3D.X);
      cameraPose.getOrientation().transform(euclidDirection);
      euclidUp.setIncludingFrame(ReferenceFrame.getWorldFrame(), Axis3D.Z);
      cameraPose.getOrientation().transform(euclidUp);

      LibGDXTools.toLibGDX(cameraPose.getPosition(), position);
      LibGDXTools.toLibGDX(euclidDirection, direction);
      LibGDXTools.toLibGDX(euclidUp, up);

      cameraPose.get(transformToParent);
      cameraFrame.update();
   }

   public void processImGuiInput(ImGui3DViewInput input)
   {
      isWPressed = input.isWindowHovered() && ImGui.isKeyDown('W');
      isSPressed = input.isWindowHovered() && ImGui.isKeyDown('S');
      isAPressed = input.isWindowHovered() && ImGui.isKeyDown('A');
      isDPressed = input.isWindowHovered() && ImGui.isKeyDown('D');
      isQPressed = input.isWindowHovered() && ImGui.isKeyDown('Q');
      isZPressed = input.isWindowHovered() && ImGui.isKeyDown('Z');
      isCPressed = input.isWindowHovered() && ImGui.isKeyDown('C');
      isEPressed = input.isWindowHovered() && ImGui.isKeyDown('E');

      int orbitMouseButton = useMiddleClickViewOrbit ? ImGuiMouseButton.Middle : ImGuiMouseButton.Left;
      ImGuiMouseDragData orbitDragData = input.getMouseDragData(orbitMouseButton);
      boolean dragJustStarted = ImGui.isWindowHovered() && orbitDragData.getDragJustStarted();

      ImGuiMouseDragData panDragData = !useMiddleClickViewOrbit ? input.getMouseDragData(ImGuiMouseButton.Middle) : null;
      boolean panDrag = ImGui.isWindowHovered() && panDragData != null && panDragData.getDragJustStarted();

      if (!useMiddleClickViewOrbit)
         dragJustStarted &= input.getClosestPick() == null;
      if (dragJustStarted)
      {
         orbitDragData.setObjectBeingDragged(this);
      }
      if (orbitDragData.isBeingDragged(this))
      {
         mouseDragged(orbitDragData.getMouseDraggedX(), orbitDragData.getMouseDraggedY());
      }

      if (panDrag)
      {
         panDragData.setObjectBeingDragged(this);
      }
      if (panDragData != null && panDragData.isBeingDragged(this))
      {
         mousePanned(panDragData.getMouseDraggedX(), panDragData.getMouseDraggedY());
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

   /**
    * TODO: This should pan along the plane orthogonal to the camera and on the focus point.
    * TODO: It should also scale in speed w.r.t. zoom.
    */
   private void mousePanned(float deltaX, float deltaY)
   {
      focusPointPose.appendTranslation(0.0, 0.005 * deltaX, 0.0);
      focusPointPose.appendTranslation(0.005 * deltaY, 0.0, 0.0);
      updateCameraPose();
   }

   private void scrolled(float amountY)
   {
      zoom = zoom + Math.signum(amountY) * zoom * zoomSpeedFactor;
   }

   // Taken from GDX PerspectiveCamera

   @Override
   public void update()
   {
      if (inputEnabled)
      {
         float deltaTime = Gdx.app.getGraphics().getDeltaTime();

         if (libGDXInputMode)
         {
            isWPressed = Gdx.input.isKeyPressed(Input.Keys.W);
            isSPressed = Gdx.input.isKeyPressed(Input.Keys.S);
            isAPressed = Gdx.input.isKeyPressed(Input.Keys.A);
            isDPressed = Gdx.input.isKeyPressed(Input.Keys.D);
            isQPressed = Gdx.input.isKeyPressed(Input.Keys.Q);
            isZPressed = Gdx.input.isKeyPressed(Input.Keys.Z);
            isCPressed = Gdx.input.isKeyPressed(Input.Keys.C);
            isEPressed = Gdx.input.isKeyPressed(Input.Keys.E);
         }

         boolean ctrlHeld = ImGui.getIO().getKeyCtrl();
         boolean shiftHeld = ImGui.getIO().getKeyShift();
         if(ctrlHeld)
         {
            double zoomAmount = 5.0 * deltaTime;
            double orbitAmount = 500.0 * deltaTime;
            if (shiftHeld)
            {
               zoomAmount *= 0.2;
               orbitAmount *= 0.2;
            }

            if (isEPressed)
               zoom -= zoomAmount;
            if (isCPressed)
               zoom += zoomAmount;
            if (isAPressed)
               longitude += longitudeSpeed * orbitAmount;
            if (isDPressed)
               longitude -= longitudeSpeed * orbitAmount;
            if (isSPressed)
               latitude += latitudeSpeed * orbitAmount;
            if (isWPressed)
               latitude -= latitudeSpeed * orbitAmount;
         }
         else
         {
            double translateAmount = translateSpeedFactor * zoom * deltaTime;
            if (shiftHeld)
               translateAmount *= 0.2;
            if (isWPressed)
            {
               focusPointPose.appendTranslation(translateAmount, 0.0, 0.0);
            }
            if (isSPressed)
            {
               focusPointPose.appendTranslation(-translateAmount, 0.0, 0.0);
            }
            if (isAPressed)
            {
               focusPointPose.appendTranslation(0.0, translateAmount, 0.0);
            }
            if (isDPressed)
            {
               focusPointPose.appendTranslation(0.0, -translateAmount, 0.0);
            }
            if (isQPressed)
            {
               focusPointPose.appendTranslation(0.0, 0.0, translateAmount);
            }
            if (isZPressed)
            {
               focusPointPose.appendTranslation(0.0, 0.0, -translateAmount);
            }
         }
         updateCameraPose();
      }

      update(true);
   }

   final Vector3 tmp = new Vector3();

   /** https://glprogramming.com/red/appendixf.html */
   @Override
   public void update(boolean updateFrustum)
   {
      float widthOverHeightRatio = viewportWidth / viewportHeight;
      projection.setToProjection(Math.abs(near), Math.abs(far), verticalFieldOfView, widthOverHeightRatio);
      // TODO: It'd be nice to switch to projection and view matrices with friendlier frames
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

   public void setPose(RigidBodyTransform transformToWorld)
   {
      cameraPose.set(transformToWorld);

      euclidDirection.setIncludingFrame(ReferenceFrame.getWorldFrame(), Axis3D.X);
      cameraPose.getOrientation().transform(euclidDirection);
      euclidUp.setIncludingFrame(ReferenceFrame.getWorldFrame(), Axis3D.Z);
      cameraPose.getOrientation().transform(euclidUp);

      LibGDXTools.toLibGDX(cameraPose.getPosition(), position);
      LibGDXTools.toLibGDX(euclidDirection, direction);
      LibGDXTools.toLibGDX(euclidUp, up);

      cameraPose.get(transformToParent);
      cameraFrame.update();
   }

   public void dispose()
   {
      focusPointModel.dispose();
   }

   public ReferenceFrame getCameraFrame()
   {
      return cameraFrame;
   }

   public FramePose3D getCameraPose()
   {
      return cameraPose;
   }

   public FramePose3DReadOnly getFocusPointPose()
   {
      return focusPointPose;
   }

   public void setInputEnabled(boolean inputEnabled)
   {
      this.inputEnabled = inputEnabled;
   }

   public void setVerticalFieldOfView(double verticalFieldOfView)
   {
      this.verticalFieldOfView = (float) verticalFieldOfView;
   }

   public float getVerticalFieldOfView()
   {
      return verticalFieldOfView;
   }

   public void setUseMiddleClickViewOrbit(boolean useMiddleClickViewOrbit)
   {
      this.useMiddleClickViewOrbit = useMiddleClickViewOrbit;
   }

   public void setFocusPointPose(RigidBodyTransform rigidBodyTransform)
   {
      focusPointPose.set(rigidBodyTransform);
      updateCameraPose();
   }
}