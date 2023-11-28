package us.ihmc.rdx.ui.gizmo;

import imgui.internal.ImGui;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.imgui.ImGuiInputDouble;
import us.ihmc.rdx.imgui.ImGuiInputDoubleForRotations;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.tools.UnitConversions;

import java.util.function.Supplier;

/**
 * This class provides functionality for modifying a pose gizmo
 * with respect to several reference frames. It provides functionality
 * needed to click and drag the gizmos in different ways,
 * to set them to values using number input widgets, and to
 * nudge them around by incremental amounts.
 */
public class FrameBasedGizmoModification
{
   private static final boolean SET_ABSOLUTE = false;
   public static final boolean PREPEND = true;
   private static final double TRANSLATION_EPSILON = 1e-8;
   /** Any less than this and the gizmo will report motion even when the mouse is not moving. */
   private static final double ROTATION_EPSILON = 1e-4;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final boolean yawOnly;
   private final Supplier<ReferenceFrame> gizmoFrameSupplier;
   private final Supplier<ReferenceFrame> parentReferenceFrameSupplier;
   private final RDXFocusBasedCamera camera3D;

   /** Pose used for making adjustments. */
   private final FramePose3D adjustmentPose3D = new FramePose3D();
   /** Used for adjusting orientation with respect to different frames. */
   private final FrameQuaternion rotationAdjustmentQuaternion = new FrameQuaternion();
   /** Used to move the gizmo with respect to where you are looking at the scene from. */
   private final MutableReferenceFrame cameraZUpFrameForAdjustment = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final ImGuiInputDouble translationStepSizeInput;
   private final ImGuiInputDouble positionXImGuiInput;
   private final ImGuiInputDouble positionYImGuiInput;
   private final ImGuiInputDouble positionZImGuiInput;
   private final ImGuiInputDouble rotationStepSizeInput;
   private final ImGuiInputDoubleForRotations yawImGuiInput;
   private final ImGuiInputDoubleForRotations pitchImGuiInput;
   private final ImGuiInputDoubleForRotations rollImGuiInput;

   private boolean adjustmentNeedsToBeApplied = false;
   private RDXPose3DGizmoAdjustmentFrame translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
   private RDXPose3DGizmoAdjustmentFrame rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;

   public FrameBasedGizmoModification(Supplier<ReferenceFrame> gizmoFrameSupplier,
                                      Supplier<ReferenceFrame> parentReferenceFrameSupplier,
                                      RDXFocusBasedCamera camera3D)
   {
      this(gizmoFrameSupplier, parentReferenceFrameSupplier, camera3D, false);
   }

   public FrameBasedGizmoModification(Supplier<ReferenceFrame> gizmoFrameSupplier,
                                      Supplier<ReferenceFrame> parentReferenceFrameSupplier,
                                      RDXFocusBasedCamera camera3D,
                                      boolean yawOnly)
   {
      this.yawOnly = yawOnly;
      this.gizmoFrameSupplier = gizmoFrameSupplier;
      this.parentReferenceFrameSupplier = parentReferenceFrameSupplier;
      this.camera3D = camera3D;

      translationStepSizeInput = new ImGuiInputDouble("Translation step size", "%.5f", 0.01);
      positionXImGuiInput = new ImGuiInputDouble("X", "%.5f");
      positionYImGuiInput = new ImGuiInputDouble("Y", "%.5f");
      positionZImGuiInput = new ImGuiInputDouble("Z", "%.5f");
      rotationStepSizeInput = new ImGuiInputDouble("Rotation step size " + UnitConversions.DEGREE_SYMBOL, "%.5f", 0.5);
      yawImGuiInput = new ImGuiInputDoubleForRotations("Yaw " + UnitConversions.DEGREE_SYMBOL, "%.5f");
      if (!yawOnly)
      {
         pitchImGuiInput = new ImGuiInputDoubleForRotations("Pitch " + UnitConversions.DEGREE_SYMBOL, "%.5f");
         rollImGuiInput = new ImGuiInputDoubleForRotations("Roll " + UnitConversions.DEGREE_SYMBOL, "%.5f");
      }
      else
      {
         pitchImGuiInput = null;
         rollImGuiInput = null;
      }
   }

   public void translateInWorld(Vector3DReadOnly translation)
   {
      if (translation.normSquared() > TRANSLATION_EPSILON)
      {
         adjustmentPose3D.changeFrame(adjustmentPose3D.getReferenceFrame().getRootFrame());
         adjustmentPose3D.getPosition().add(translation);
         adjustmentNeedsToBeApplied = true;
      }
   }

   public void rotateInWorld(AxisAngleReadOnly rotationInWorld)
   {
      if (Math.abs(rotationInWorld.angle()) > ROTATION_EPSILON)
      {
         adjustmentPose3D.changeFrame(adjustmentPose3D.getReferenceFrame().getRootFrame());
         rotationInWorld.transform(adjustmentPose3D.getOrientation());
         adjustmentNeedsToBeApplied = true;
      }
   }

   public void yawInWorld(double yaw)
   {
      if (Math.abs(yaw) > ROTATION_EPSILON)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.appendYawRotation(yaw);
         afterRotationAdjustment(PREPEND);
         adjustmentNeedsToBeApplied = true;
      }
   }

   /**
    * @return If adjustment was applied.
    */
   public boolean applyAdjustmentIfNeeded(RigidBodyTransform transformToParentToPack)
   {
      boolean adjustmentApplied = adjustmentNeedsToBeApplied;
      if (adjustmentNeedsToBeApplied)
      {
         adjustmentNeedsToBeApplied = false;
         adjustmentPose3D.changeFrame(parentReferenceFrameSupplier.get());
         adjustmentPose3D.get(transformToParentToPack);
      }
      return adjustmentApplied;
   }

   public void setToZeroGizmoFrame()
   {
      adjustmentPose3D.setToZero(gizmoFrameSupplier.get());
   }

   public void renderImGuiTunerWidgets()
   {
      ReferenceFrame parentReferenceFrame = parentReferenceFrameSupplier.get();

      ImGui.text("Parent frame: " + parentReferenceFrame.getName());

      ImGui.text("Translation adjustment frame:");
      if (ImGui.radioButton(labels.get("Camera Z Up", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Camera", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.WORLD;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Parent", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.PARENT;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Local", 0), translationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.LOCAL))
         translationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.LOCAL;
      ImGui.text("Rotation adjustment frame:");
      if (ImGui.radioButton(labels.get("Camera Z Up", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Camera", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.CAMERA;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.WORLD;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Parent", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.PARENT;
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Local", 1), rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.LOCAL))
         rotationAdjustmentFrame = RDXPose3DGizmoAdjustmentFrame.LOCAL;

      beforeForTranslationAdjustment();
      translationStepSizeInput.render(RDXGizmoTools.INITIAL_SMALL_STEP, RDXGizmoTools.INITIAL_BIG_STEP);
      positionXImGuiInput.setDoubleValue(adjustmentPose3D.getPosition().getX());
      positionYImGuiInput.setDoubleValue(adjustmentPose3D.getPosition().getY());
      positionZImGuiInput.setDoubleValue(adjustmentPose3D.getPosition().getZ());
      adjustmentNeedsToBeApplied |= positionXImGuiInput.render(translationStepSizeInput.getDoubleValue(),
                                                               RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * translationStepSizeInput.getDoubleValue());
      adjustmentNeedsToBeApplied |= positionYImGuiInput.render(translationStepSizeInput.getDoubleValue(),
                                                               RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * translationStepSizeInput.getDoubleValue());
      adjustmentNeedsToBeApplied |= positionZImGuiInput.render(translationStepSizeInput.getDoubleValue(),
                                                               RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * translationStepSizeInput.getDoubleValue());
      adjustmentPose3D.getPosition().set(positionXImGuiInput.getDoubleValue(), positionYImGuiInput.getDoubleValue(), positionZImGuiInput.getDoubleValue());

      rotationStepSizeInput.render(RDXGizmoTools.INITIAL_FINE_ROTATION, RDXGizmoTools.INITIAL_COARSE_ROTATION);
      Orientation3DBasics orientationToPrint = getOrientationInAdjustmentFrame();
      yawImGuiInput.setDoubleValue(Math.toDegrees(orientationToPrint.getYaw()));
      if (!yawOnly)
      {
         pitchImGuiInput.setDoubleValue(Math.toDegrees(orientationToPrint.getPitch()));
         rollImGuiInput.setDoubleValue(Math.toDegrees(orientationToPrint.getRoll()));
      }
      yawImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
      if (!yawOnly)
      {
         pitchImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
         rollImGuiInput.render(rotationStepSizeInput.getDoubleValue(), RDXGizmoTools.FINE_TO_COARSE_MULTIPLIER * rotationStepSizeInput.getDoubleValue());
      }
      boolean inputChanged = yawImGuiInput.getInputChanged();
      if (!yawOnly)
      {
         inputChanged |= pitchImGuiInput.getInputChanged();
         inputChanged |= rollImGuiInput.getInputChanged();
      }
      adjustmentNeedsToBeApplied |= inputChanged;
      if (inputChanged)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.setYawPitchRoll(Math.toRadians(yawImGuiInput.getDoubleValue()),
                                             yawOnly ? 0.0 : Math.toRadians(pitchImGuiInput.getDoubleValue()),
                                             yawOnly ? 0.0 : Math.toRadians(rollImGuiInput.getDoubleValue()));
         afterRotationAdjustment(SET_ABSOLUTE);
      }
      boolean rotationStepped = yawImGuiInput.getStepButtonClicked();
      if (!yawOnly)
      {
         rotationStepped |= pitchImGuiInput.getStepButtonClicked();
         rotationStepped |= rollImGuiInput.getStepButtonClicked();
      }
      adjustmentNeedsToBeApplied |= rotationStepped;
      if (rotationStepped)
      {
         Orientation3DBasics orientationToAdjust = beforeForRotationAdjustment();
         orientationToAdjust.setYawPitchRoll(Math.toRadians(yawImGuiInput.getSteppedAmount()),
                                             yawOnly ? 0.0 : Math.toRadians(pitchImGuiInput.getSteppedAmount()),
                                             yawOnly ? 0.0 : Math.toRadians(rollImGuiInput.getSteppedAmount()));
         afterRotationAdjustment(PREPEND);
      }

      ImGui.text("Set to zero in:");
      ImGui.sameLine();
      if (ImGui.button("World"))
      {
         adjustmentPose3D.setToZero(ReferenceFrame.getWorldFrame());
         adjustmentNeedsToBeApplied = true;
      }
      ImGui.sameLine();
      if (ImGui.button("Parent"))
      {
         adjustmentPose3D.setToZero(parentReferenceFrame);
         adjustmentNeedsToBeApplied = true;
      }
   }

   public Point3DBasics beforeForTranslationAdjustment()
   {
      ReferenceFrame parentReferenceFrame = parentReferenceFrameSupplier.get();

      switch (translationAdjustmentFrame)
      {
         case WORLD -> adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
         case PARENT -> adjustmentPose3D.changeFrame(parentReferenceFrame);
         case CAMERA_ZUP ->
         {
            prepareCameraZUpFrameForAdjustment();
            adjustmentPose3D.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
         }
         case CAMERA ->
         {
            adjustmentPose3D.changeFrame(camera3D.getCameraFrame());
         }
      }

      return adjustmentPose3D.getPosition();
   }

   /**
    * Used to show the user the current rotation with respect to the current adjustment frame.
    */
   private Orientation3DBasics getOrientationInAdjustmentFrame()
   {
      switch (rotationAdjustmentFrame)
      {
         case WORLD, PARENT, CAMERA, CAMERA_ZUP ->
         {
            if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD)
            {
               adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT)
            {
               adjustmentPose3D.changeFrame(parentReferenceFrameSupplier.get());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP)
            {
               prepareCameraZUpFrameForAdjustment();
               adjustmentPose3D.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
            }
            else // CAMERA
            {
               adjustmentPose3D.changeFrame(camera3D.getCameraFrame());
            }
         }
         default -> // LOCAL
         {
            adjustmentPose3D.changeFrame(gizmoFrameSupplier.get());
         }
      }
      return adjustmentPose3D.getOrientation();
   }

   /**
    * Call before performing an adjustment of this pose's orientation.
    * @return orientation to adjust
    */
   public Orientation3DBasics beforeForRotationAdjustment()
   {
      switch (rotationAdjustmentFrame)
      {
         case WORLD, PARENT, CAMERA, CAMERA_ZUP ->
         {
            if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD)
            {
               rotationAdjustmentQuaternion.setToZero(ReferenceFrame.getWorldFrame());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT)
            {
               rotationAdjustmentQuaternion.setToZero(parentReferenceFrameSupplier.get());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP)
            {
               prepareCameraZUpFrameForAdjustment();
               rotationAdjustmentQuaternion.setToZero(cameraZUpFrameForAdjustment.getReferenceFrame());
            }
            else // CAMERA
            {
               rotationAdjustmentQuaternion.setToZero(camera3D.getCameraFrame());
            }

            return rotationAdjustmentQuaternion;
         }
         default -> // LOCAL
         {
            adjustmentPose3D.changeFrame(gizmoFrameSupplier.get());
            return adjustmentPose3D.getOrientation();
         }
      }
   }

   private void prepareCameraZUpFrameForAdjustment()
   {
      cameraZUpFrameForAdjustment.getTransformToParent().getTranslation().set(camera3D.getCameraPose().getPosition());
      cameraZUpFrameForAdjustment.getTransformToParent().getRotation().setToYawOrientation(camera3D.getCameraPose().getYaw());
      cameraZUpFrameForAdjustment.getReferenceFrame().update();
   }

   /**
    * Call after performing an adjustment on the orientation returned by {@link #beforeForRotationAdjustment}
    * @param prepend true if the adjustment is meant to be a nudge, false if the adjustment is meant to fully replace
    *                the current orientation. Use the constants {@link #SET_ABSOLUTE} and {@link #PREPEND}.
    */
   public void afterRotationAdjustment(boolean prepend)
   {
      switch (rotationAdjustmentFrame)
      {
         case WORLD, PARENT, CAMERA, CAMERA_ZUP ->
         {
            if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.WORLD)
            {
               rotationAdjustmentQuaternion.changeFrame(ReferenceFrame.getWorldFrame());
               adjustmentPose3D.changeFrame(ReferenceFrame.getWorldFrame());
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.PARENT)
            {
               ReferenceFrame parentReferenceFrame = parentReferenceFrameSupplier.get();
               rotationAdjustmentQuaternion.changeFrame(parentReferenceFrame);
               adjustmentPose3D.changeFrame(parentReferenceFrame);
            }
            else if (rotationAdjustmentFrame == RDXPose3DGizmoAdjustmentFrame.CAMERA_ZUP)
            {
               rotationAdjustmentQuaternion.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
               adjustmentPose3D.changeFrame(cameraZUpFrameForAdjustment.getReferenceFrame());
            }
            else // CAMERA
            {
               rotationAdjustmentQuaternion.changeFrame(camera3D.getCameraFrame());
               adjustmentPose3D.changeFrame(camera3D.getCameraFrame());
            }
            if (prepend == PREPEND)
               adjustmentPose3D.getOrientation().prepend(rotationAdjustmentQuaternion);
            else // SET_ABSOLUTE
               adjustmentPose3D.getOrientation().set(rotationAdjustmentQuaternion);
         }
      }
   }

   public void setAdjustmentNeedsToBeApplied()
   {
      adjustmentNeedsToBeApplied = true;
   }
}
