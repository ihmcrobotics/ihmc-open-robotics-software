package us.ihmc.footstepPlanning.ui.components;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.animation.AnimationTimer;
import javafx.scene.SubScene;
import javafx.scene.input.KeyCode;
import javafx.scene.transform.Transform;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UIStepAdjustmentFrame;
import us.ihmc.messager.Messager;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ManualFootstepAdjustmentListener extends AnimationTimer
{
   private static final double NORMAL_SPEED = 0.04;
   private static final double SLOW_SPEED = 0.01;
   private static final double YAW_MULTIPLIER = 10.0;

   private final Messager messager;
   private final SubScene subScene;
   private final List<Pose3D> manualStepAdjustments = new ArrayList<>();

   private final AtomicReference<Pair<Integer, FootstepDataMessage>> selectedStep;

   private final Pose3D footstepPose = new Pose3D();
   private int selectedStepIndex = -1;
   private UIStepAdjustmentFrame frame = UIStepAdjustmentFrame.getDefault();

   private final AtomicBoolean controlPressed = new AtomicBoolean();
   private final AtomicBoolean shiftPressed = new AtomicBoolean();
   private final AtomicBoolean tPressed = new AtomicBoolean();
   private final AtomicBoolean rPressed = new AtomicBoolean();

   private final AtomicDouble topLeftRightNumPad = new AtomicDouble();
   private final AtomicDouble middleLeftRightNumPad = new AtomicDouble();
   private final AtomicDouble upDownNumPad = new AtomicDouble();

   private long previousTimestamp;

   public ManualFootstepAdjustmentListener(Messager messager, SubScene subScene)
   {
      this.messager = messager;
      this.subScene = subScene;
      this.selectedStep = messager.createInput(FootstepPlannerMessagerAPI.SelectedFootstep, Pair.of(-1, null));

      messager.addTopicListener(FootstepPlannerMessagerAPI.GlobalReset, r -> reset());
      messager.addTopicListener(FootstepPlannerMessagerAPI.ComputePath, r -> reset());

      for (int i = 0; i < 200; i++)
      {
         manualStepAdjustments.add(new Pose3D());
      }

      subScene.setOnKeyPressed(keyEvent ->
                               {
                                  KeyCode keyCode = keyEvent.getCode();
                                  if (keyCode == KeyCode.SHIFT)
                                     shiftPressed.set(true);
                                  if (keyCode == KeyCode.CONTROL)
                                     controlPressed.set(true);
                                  if (keyCode == KeyCode.NUMPAD7)
                                     topLeftRightNumPad.set(1.0);
                                  if (keyCode == KeyCode.NUMPAD9)
                                     topLeftRightNumPad.set(-1.0);
                                  if (keyCode == KeyCode.NUMPAD4)
                                     middleLeftRightNumPad.set(1.0);
                                  if (keyCode == KeyCode.NUMPAD6)
                                     middleLeftRightNumPad.set(-1.0);
                                  if (keyCode == KeyCode.NUMPAD8)
                                     upDownNumPad.set(1.0);
                                  if (keyCode == KeyCode.NUMPAD2)
                                     upDownNumPad.set(-1.0);

                                  if (keyCode == KeyCode.T)
                                     tPressed.set(true);
                                  if (keyCode == KeyCode.R)
                                     rPressed.set(true);
                               });

      subScene.setOnKeyReleased(keyEvent ->
                                {
                                   KeyCode keyCode = keyEvent.getCode();
                                   if (keyCode == KeyCode.SHIFT)
                                      shiftPressed.set(false);
                                   if (keyCode == KeyCode.CONTROL)
                                      controlPressed.set(false);
                                   if (keyCode == KeyCode.NUMPAD7 || keyCode == KeyCode.NUMPAD9)
                                      topLeftRightNumPad.set(0.0);
                                   if (keyCode == KeyCode.NUMPAD4 || keyCode == KeyCode.NUMPAD6)
                                      middleLeftRightNumPad.set(0.0);
                                   if (keyCode == KeyCode.NUMPAD8 || keyCode == KeyCode.NUMPAD2)
                                      upDownNumPad.set(0.0);
                                });
   }

   @Override
   public void handle(long now)
   {
      handleFrameChange();

      long timestamp = System.currentTimeMillis();
      long timestampDifference = timestamp - previousTimestamp;
      previousTimestamp = timestamp;

      Pair<Integer, FootstepDataMessage> selectedStep;
      selectedStepIndex = -1;

      if ((selectedStep = this.selectedStep.getAndSet(null)) != null)
      {
         int selectedStepIndex = selectedStep.getKey();
         this.selectedStepIndex = selectedStepIndex;

         if (selectedStepIndex == -1)
         {
            return;
         }

         FootstepDataMessage selectedStepPose = selectedStep.getValue();
         footstepPose.set(selectedStepPose.getLocation(), selectedStepPose.getOrientation());
      }

      if (selectedStepIndex == -1)
      {
         return;
      }

      double displacement = (shiftPressed.get() ? SLOW_SPEED : NORMAL_SPEED) * Conversions.millisecondsToSeconds(timestampDifference);

      boolean adjustmentRequested = shiftFootOrientation(displacement);

      if (frame == UIStepAdjustmentFrame.WORLD)
      {
         if (controlPressed.get())
         {
            adjustmentRequested = upDownNumPad.get() != 0.0;
            footstepPose.getPosition().addZ(displacement * upDownNumPad.get());
         }
         else
         {
            adjustmentRequested |= shiftFootTranslationWorld(displacement);
         }
      }
      else
      {
         if (controlPressed.get())
         {
            footstepPose.appendTranslation(0.0, 0.0, displacement * upDownNumPad.get());
            adjustmentRequested |= upDownNumPad.get() != 0.0;
         }
         else
         {
            adjustmentRequested |= shiftFootTranslationInLocal(displacement);
         }
      }

      if (adjustmentRequested)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.ManuallyAdjustmentedStep, Pair.of(selectedStepIndex, footstepPose));
      }
  }

   private boolean shiftFootOrientation(double displacement)
   {
      double deltaYaw = YAW_MULTIPLIER * topLeftRightNumPad.get() * displacement;
      footstepPose.getOrientation().appendYawRotation(deltaYaw);
      return topLeftRightNumPad.get() != 0.0;
   }

   private boolean shiftFootTranslationWorld(double displacement)
   {
      boolean adjustmentRequested = upDownNumPad.get() != 0.0 || middleLeftRightNumPad.get() != 0.0;
      if (!adjustmentRequested)
      {
         return false;
      }

      Transform cameraTransform = subScene.getCamera().getLocalToSceneTransform();
      Vector2D cameraToFoot = new Vector2D(footstepPose.getX() - cameraTransform.getTx(), footstepPose.getY() - cameraTransform.getTy());
      cameraToFoot.normalize();
      Vector2D orthogonalMotion = new Vector2D(-cameraToFoot.getY(), cameraToFoot.getX());

      Vector3D adjustment = new Vector3D();
      adjustment.addX(cameraToFoot.getX() * upDownNumPad.get());
      adjustment.addY(cameraToFoot.getY() * upDownNumPad.get());
      adjustment.addX(orthogonalMotion.getX() * middleLeftRightNumPad.get());
      adjustment.addY(orthogonalMotion.getY() * middleLeftRightNumPad.get());
      adjustment.scale(displacement);

      footstepPose.getPosition().add(adjustment);
      return true;
   }

   private boolean shiftFootTranslationInLocal(double displacement)
   {
      boolean adjustmentRequested = upDownNumPad.get() != 0.0 || middleLeftRightNumPad.get() != 0.0;
      if (!adjustmentRequested)
      {
         return false;
      }

      Plane3D footPlane = new Plane3D();
      footPlane.getPoint().set(footstepPose.getPosition());
      footstepPose.getOrientation().transform(footPlane.getNormal());

      Transform cameraTransform = subScene.getCamera().getLocalToSceneTransform();
      Point3D cameraPosition = new Point3D(cameraTransform.getTx(), cameraTransform.getTy(), cameraTransform.getTz());
      footPlane.orthogonalProjection(cameraPosition);

      Vector3D cameraToFoot = new Vector3D(footstepPose.getPosition());
      cameraToFoot.sub(cameraPosition);
      cameraToFoot.normalize();

      Vector3D orthogonalMotion = new Vector3D();
      orthogonalMotion.cross(footPlane.getNormal(), cameraToFoot);

      Vector3D adjustment = new Vector3D();

      adjustment.addX(cameraToFoot.getX() * upDownNumPad.get());
      adjustment.addY(cameraToFoot.getY() * upDownNumPad.get());
      adjustment.addZ(cameraToFoot.getZ() * upDownNumPad.get());

      adjustment.addX(orthogonalMotion.getX() * middleLeftRightNumPad.get());
      adjustment.addY(orthogonalMotion.getY() * middleLeftRightNumPad.get());
      adjustment.addZ(orthogonalMotion.getZ() * middleLeftRightNumPad.get());

      adjustment.scale(displacement);
      footstepPose.getPosition().add(adjustment);

      return true;
   }

   private void handleFrameChange()
   {
      if (rPressed.getAndSet(false))
      {
         frame = frame.getOppositeMode();
         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepAdjustmentFrame, frame);
      }
   }

   private void reset()
   {
      for (int i = 0; i < manualStepAdjustments.size(); i++)
      {
         manualStepAdjustments.get(i).setToZero();
      }
   }
}
