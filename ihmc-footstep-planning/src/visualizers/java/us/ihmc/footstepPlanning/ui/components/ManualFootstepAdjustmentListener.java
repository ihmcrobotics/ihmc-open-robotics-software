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
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UIStepAdjustmentFrame;
import us.ihmc.footstepPlanning.communication.UIStepAdjustmentMode;
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
   private UIStepAdjustmentMode mode = UIStepAdjustmentMode.TRANSLATION;
   private UIStepAdjustmentFrame frame = UIStepAdjustmentFrame.WORLD;

   private final AtomicBoolean controlPressed = new AtomicBoolean();
   private final AtomicBoolean shiftPressed = new AtomicBoolean();
   private final AtomicBoolean tPressed = new AtomicBoolean();
   private final AtomicBoolean rPressed = new AtomicBoolean();

   private final AtomicDouble leftRightArrowValue = new AtomicDouble();
   private final AtomicDouble upDownArrowValue = new AtomicDouble();

   private long previousTimestamp;

   public ManualFootstepAdjustmentListener(Messager messager, SubScene subScene)
   {
      this.messager = messager;
      this.subScene = subScene;
      this.selectedStep = messager.createInput(FootstepPlannerMessagerAPI.SelectedFootstep, Pair.of(-1, null));

      messager.registerTopicListener(FootstepPlannerMessagerAPI.GlobalReset, r -> reset());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, r -> reset());

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
                                  if (keyCode == KeyCode.NUMPAD4)
                                     leftRightArrowValue.set(1.0);
                                  if (keyCode == KeyCode.NUMPAD6)
                                     leftRightArrowValue.set(-1.0);
                                  if (keyCode == KeyCode.NUMPAD8)
                                     upDownArrowValue.set(1.0);
                                  if (keyCode == KeyCode.NUMPAD2)
                                     upDownArrowValue.set(-1.0);

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
                                   if (keyCode == KeyCode.NUMPAD4 || keyCode == KeyCode.NUMPAD6)
                                      leftRightArrowValue.set(0.0);
                                   if (keyCode == KeyCode.NUMPAD8 || keyCode == KeyCode.NUMPAD2)
                                      upDownArrowValue.set(0.0);
                                });
   }

   @Override
   public void handle(long now)
   {
      handleModeAndFrameChange();

      long timestamp = System.currentTimeMillis();
      long timestampDifference = timestamp - previousTimestamp;
      previousTimestamp = timestamp;

      Pair<Integer, FootstepDataMessage> selectedStep;
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

      if (frame == UIStepAdjustmentFrame.WORLD)
      {
         if (mode == UIStepAdjustmentMode.ORIENTATION)
         {
            double deltaYaw = YAW_MULTIPLIER * leftRightArrowValue.get() * displacement;
            footstepPose.getOrientation().appendYawRotation(deltaYaw);
         }
         else
         {
            if (controlPressed.get())
            {
               footstepPose.getPosition().addZ(displacement * upDownArrowValue.get());
            }
            else
            {
               Transform cameraTransform = subScene.getCamera().getLocalToSceneTransform();
               Vector2D cameraToFoot = new Vector2D(footstepPose.getX() - cameraTransform.getTx(), footstepPose.getY() - cameraTransform.getTy());
               cameraToFoot.normalize();
               Vector2D orthogonalMotion = new Vector2D(-cameraToFoot.getY(), cameraToFoot.getX());

               Vector3D adjustment = new Vector3D();
               adjustment.addX(cameraToFoot.getX() * upDownArrowValue.get());
               adjustment.addY(cameraToFoot.getY() * upDownArrowValue.get());
               adjustment.addX(orthogonalMotion.getX() * leftRightArrowValue.get());
               adjustment.addY(orthogonalMotion.getY() * leftRightArrowValue.get());
               adjustment.scale(displacement);

               footstepPose.getPosition().add(adjustment);
            }
         }
      }
      else
      {
         if (mode == UIStepAdjustmentMode.ORIENTATION)
         {
            double deltaYaw = YAW_MULTIPLIER * leftRightArrowValue.get() * displacement;
            footstepPose.getOrientation().appendYawRotation(deltaYaw);
         }
         else
         {
            if (controlPressed.get())
            {
               footstepPose.appendTranslation(0.0, 0.0, displacement * upDownArrowValue.get());
            }
            else
            {
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

               adjustment.addX(cameraToFoot.getX() * upDownArrowValue.get());
               adjustment.addY(cameraToFoot.getY() * upDownArrowValue.get());
               adjustment.addZ(cameraToFoot.getZ() * upDownArrowValue.get());

               adjustment.addX(orthogonalMotion.getX() * leftRightArrowValue.get());
               adjustment.addY(orthogonalMotion.getY() * leftRightArrowValue.get());
               adjustment.addZ(orthogonalMotion.getZ() * leftRightArrowValue.get());

               adjustment.scale(displacement);

               footstepPose.getPosition().add(adjustment);
            }
         }
      }

      messager.submitMessage(FootstepPlannerMessagerAPI.ManuallyAdjustmentedStep, Pair.of(selectedStepIndex, footstepPose));
   }

   private void handleModeAndFrameChange()
   {
      if (tPressed.getAndSet(false))
      {
         mode = mode.getOppositeMode();
         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepAdjustmentMode, mode);
      }

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
