package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.InputAnalogActionData;
import org.lwjgl.openvr.InputDigitalActionData;
import org.lwjgl.openvr.InputOriginInfo;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRInput;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.imgui.ImGuiRigidBodyTransformTuner;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * 1. Add action to actions.json (also give it a fancy name at the bottom)
 * 2. Launch the RDX VR UI
 * 3. Hit the Steam menu button
 * 4. Go into "Old Controller Bindings UI"
 * 5. Edit the bindings
 * 6. Go back, exit steam menu
 * 7. Copy newly generated bindings from Documents\steamvr\input
 *    into index_bindings.json
 * 8. Add programmatic fields and getters to this class for the new action,
 *    by pattern matching what's here
 *
 * It's possible to do this for multiple types of controllers.
 * Currently, we have mappings for Valve Index and Vive Focus 3 controllers.
 * You can change the default bindings in actions.json to select the right one for the controller you're using.
 */
public class RDXVRController extends RDXVRTrackedDevice
{
   /**
    * The Valve Index controller grip is pretty sensitive. 0.7 seems like a good amount
    * of grip to not be an accident but also not require squeezing too hard.
    * The Vive Focus 3 apparenly just goes from 0.0 to 1.0 so is not a factor.
    */
   public static final double GRIP_AS_BUTTON_THRESHOLD = 0.7;
   public static final double JOYSTICK_ZERO_THRESHOLD = 0.3;

   private final RobotSide side;
   private RDXVRControllerModel model;

   private final LongBuffer inputSourceHandle = BufferUtils.newLongBuffer(1);
   private InputOriginInfo.Buffer inputOriginInfo;

   private final LongBuffer clickTriggerActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData clickTriggerActionData;
   private final LongBuffer triggerTouchedActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData triggerTouchedActionData;
   private final LongBuffer triggerActionHandle = BufferUtils.newLongBuffer(1);
   private InputAnalogActionData triggerActionData;
   private final LongBuffer aButtonActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData aButtonActionData;
   private final LongBuffer aTouchedActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData aTouchedActionData;
   private final LongBuffer bButtonActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData bButtonActionData;
   private final LongBuffer bButtonDoubleClickActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData bButtonDoubleClickActionData;
   private final LongBuffer bTouchedActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData bTouchedActionData;
   private final LongBuffer joystickPressActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData joystickPressActionData;
   private final LongBuffer touchpadTouchedActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData touchpadTouchedActionData;
   private final LongBuffer touchpadActionHandle = BufferUtils.newLongBuffer(1);
   private InputAnalogActionData touchpadActionData;
   private final LongBuffer joystickActionHandle = BufferUtils.newLongBuffer(1);
   private InputAnalogActionData joystickActionData;
   private boolean joystickIsCentered = false;
   private final RDXVRControllerRadialMenu radialMenu;
   private final LongBuffer gripActionHandle = BufferUtils.newLongBuffer(1);
   private InputAnalogActionData gripActionData;
   private boolean gripAsButtonDown = false;

   private static final RigidBodyTransformReadOnly controllerYBackZLeftXRightToXForwardZUp = new RigidBodyTransform(
      new YawPitchRoll(          // For this transformation, we start with IHMC ZUp with index forward and thumb up
         Math.toRadians(90.0),   // rotating around thumb, index goes forward to left
         Math.toRadians(135.0),  // rotating about middle finger, index goes down and a little to the right
         Math.toRadians(0.0)     // no rotation about index finger
      ),
      new Point3D()
   );
   private final ReferenceFrame xForwardZUpControllerFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempRigidBodyTransform = new RigidBodyTransform();
   /**
    * A "pick" pose in the sense of mouse picking, or colliding with the scene for selection and
    * interaction purposes. The point is just in front of the controller and represented as a little
    * sphere so the user knows where it is. Unlike a mouse, it's not just a point, but a full pose
    * which can be used to incorporate orientation into selection and interaction as well. Also,
    * picking with a mouse is usually done by casting a ray into the scene, perpendicular to the
    * monitor. However, in VR, we are not so constrained. However, we will have a ray option for the VR
    * controller as well, as it is also convenient to not have to travel to the thing you are
    * selecting or interacting with.
    */
   private final FramePose3D pickPoseFramePose = new FramePose3D();
   private final MutableReferenceFrame pickPoseFrame;
   private final ImGuiRigidBodyTransformTuner pickPoseTransformTuner;
   private final ArrayList<RDXVRPickResult> pickResults = new ArrayList<>();
   private RDXVRPickResult selectedPick;
   private final FrameLine3D pickRay = new FrameLine3D();
   private final FramePoint3D pickCollisionPoint = new FramePoint3D();
   private RDXModelInstance pickPoseSphere;
   private RDXModelInstance pickRayGraphic;
   private RDXModelInstance pickRayCollisionPointGraphic;
   private RDXVRControllerButtonLabel aButtonLabel;
   private RDXVRControllerButtonLabel bButtonLabel;
   private RDXVRControllerButtonLabel selectedPickLabel;
   private RDXVRControllerButtonLabel gripAmountLabel;
   private final RDXVRDragData triggerDragData;
   private final RDXVRDragData gripDragData;

   public RDXVRController(RDXVRControllerModel model, RobotSide side, ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      this.model = model;
      this.side = side;

      xForwardZUpControllerFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(side.getLowerCaseName() + "_xForwardZUpControllerFrame",
                                                                                getDeviceYUpZBackFrame(),
                                                                                controllerYBackZLeftXRightToXForwardZUp);
      pickPoseFrame = new MutableReferenceFrame(xForwardZUpControllerFrame);
      pickPoseFrame.getTransformToParent().getTranslation().setX(0.029);
      pickPoseFrame.getTransformToParent().getTranslation().setY(side.negateIfLeftSide(0.020));
      pickPoseFrame.getTransformToParent().getTranslation().setZ(-0.017);
      pickPoseFrame.getReferenceFrame().update();
      pickPoseTransformTuner = new ImGuiRigidBodyTransformTuner(pickPoseFrame.getTransformToParent());

      triggerDragData = new RDXVRDragData(() -> getClickTriggerActionData().bState(), pickPoseFrame.getReferenceFrame());
      gripDragData = new RDXVRDragData(this::getGripAsButtonDown, pickPoseFrame.getReferenceFrame());

      radialMenu = new RDXVRControllerRadialMenu(side, xForwardZUpControllerFrame);
   }

   public void initSystem()
   {
      inputOriginInfo = InputOriginInfo.create(1);
      VRInput.VRInput_GetInputSourceHandle("/user/hand/" + side.getLowerCaseName(), inputSourceHandle);

      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_clicktrigger", clickTriggerActionHandle);
      clickTriggerActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_triggertouched", triggerTouchedActionHandle);
      triggerTouchedActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_trigger", triggerActionHandle);
      triggerActionData = InputAnalogActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_abutton", aButtonActionHandle);
      aButtonActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_atouched", aTouchedActionHandle);
      aTouchedActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_bbutton", bButtonActionHandle);
      bButtonActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_bbuttondoubleclick", bButtonDoubleClickActionHandle);
      bButtonDoubleClickActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_btouched", bTouchedActionHandle);
      bTouchedActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_joystickpress", joystickPressActionHandle);
      joystickPressActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_touchpadtouched", touchpadTouchedActionHandle);
      touchpadTouchedActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_touchpad", touchpadActionHandle);
      touchpadActionData = InputAnalogActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_joystick", joystickActionHandle);
      joystickActionData = InputAnalogActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_grip", gripActionHandle);
      gripActionData = InputAnalogActionData.create();
   }

   public void update(RDXVRTrackedDevicePose[] trackedDevicePoses)
   {
      VRInput.VRInput_GetOriginTrackedDeviceInfo(inputSourceHandle.get(0), inputOriginInfo.get(0));
      setDeviceIndex(inputOriginInfo.trackedDeviceIndex());
      // VRSystem.VRSystem_IsTrackedDeviceConnected(leftHandDeviceIndex);
      setConnected(getDeviceIndex() != VR.k_unTrackedDeviceIndexInvalid);

      super.update(trackedDevicePoses);

      if (isConnected())
      {
         if (pickPoseSphere == null)
         {
            pickPoseSphere = new RDXModelInstance(RDXModelBuilder.createSphere(0.0025f, new Color(0x870707ff)));
            pickRayCollisionPointGraphic = new RDXModelInstance(RDXModelBuilder.createSphere(0.0015f, new Color(Color.WHITE)));
            LibGDXTools.hideGraphic(pickRayCollisionPointGraphic);

            // These were tuned by @dcalvert and @bpratt using JRebel and bringing them out of this if statement and hand tweaking
            // so they looked good for the Valve Index controllers.
            Point3D aButtonOffset = side == RobotSide.LEFT ? new Point3D(-0.085, -0.01, -0.02) : new Point3D(-0.082, -0.01, -0.017);
            Point3D bButtonOffset = side == RobotSide.LEFT ? new Point3D(-0.07, -0.013, -0.015) : new Point3D(-0.07, -0.007, -0.008);
            Point3D selectedPickLabelOffset = side == RobotSide.LEFT ? new Point3D(-0.03, 0.07, -0.004) : new Point3D(-0.04, 0.04, -0.004);
            Point3D gripAmountOffset = side == RobotSide.LEFT ? new Point3D(-0.1, -0.0, -0.07) : new Point3D(-0.1, 0.0, -0.07);
            YawPitchRoll gripAmountOrientation = side == RobotSide.LEFT ?
                  new YawPitchRoll(Math.toRadians(90.0), Math.toRadians(-37.0), Math.toRadians(90.0))
                  : new YawPitchRoll(Math.toRadians(-90.0), Math.toRadians(37.0), Math.toRadians(90.0));
            aButtonLabel = new RDXVRControllerButtonLabel(pickPoseFrame.getReferenceFrame(), side, aButtonOffset, new YawPitchRoll());
            bButtonLabel = new RDXVRControllerButtonLabel(pickPoseFrame.getReferenceFrame(), side, bButtonOffset, new YawPitchRoll());
            selectedPickLabel = new RDXVRControllerButtonLabel(pickPoseFrame.getReferenceFrame(), side, selectedPickLabelOffset, new YawPitchRoll());
            gripAmountLabel = new RDXVRControllerButtonLabel(pickPoseFrame.getReferenceFrame(), side, gripAmountOffset, gripAmountOrientation);

            radialMenu.create(joystickActionData, joystickPressActionData);
         }

         pickPoseFrame.getReferenceFrame().update();
         pickPoseFramePose.setToZero(pickPoseFrame.getReferenceFrame());
         pickPoseFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         pickPoseSphere.setPoseInWorldFrame(pickPoseFramePose);

         pickRay.setToZero(getPickPoseFrame());
         pickRay.getDirection().set(Axis3D.X);
         pickRay.changeFrame(ReferenceFrame.getWorldFrame());

         aButtonLabel.setText("");
         bButtonLabel.setText("");
         selectedPickLabel.setText("");
         gripAmountLabel.setText("%.1f".formatted(gripActionData.x()));

         radialMenu.update();
      }

      VRInput.VRInput_GetDigitalActionData(clickTriggerActionHandle.get(0), clickTriggerActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(triggerTouchedActionHandle.get(0), triggerTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(triggerActionHandle.get(0), triggerActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(aButtonActionHandle.get(0), aButtonActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(aTouchedActionHandle.get(0), aTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(bButtonActionHandle.get(0), bButtonActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(bButtonDoubleClickActionHandle.get(0), bButtonDoubleClickActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(bTouchedActionHandle.get(0), bTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(joystickPressActionHandle.get(0), joystickPressActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(touchpadActionHandle.get(0), touchpadActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(touchpadTouchedActionHandle.get(0), touchpadTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(joystickActionHandle.get(0), joystickActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(gripActionHandle.get(0), gripActionData, VR.k_ulInvalidInputValueHandle);

      gripAsButtonDown = gripActionData.x() > GRIP_AS_BUTTON_THRESHOLD;
      joystickIsCentered = Math.abs(joystickActionData.x()) < JOYSTICK_ZERO_THRESHOLD && Math.abs(joystickActionData.y()) < JOYSTICK_ZERO_THRESHOLD;

      triggerDragData.update();
      gripDragData.update();

      pickRayGraphic = null;
      if (pickRayCollisionPointGraphic != null)
         LibGDXTools.hideGraphic(pickRayCollisionPointGraphic);

      pickResults.clear();
   }

   public void updatePickResults()
   {
      selectedPick = null;

      for (RDXVRPickResult pickResult : pickResults)
      {
         if (selectedPick == null)
         {
            selectedPick = pickResult;
         }
         else if (pickResult.getDistanceToControllerPickPoint() < selectedPick.getDistanceToControllerPickPoint())
         {
            selectedPick = pickResult;
         }
      }

      if (selectedPick != null)
      {
         String labelText;
         String pickedObjectName = selectedPick.getPickedObjectName();
         if (!pickedObjectName.isEmpty())
            labelText = pickedObjectName;
         else
            labelText = selectedPick.getObjectBeingPicked() == null ? "null" : selectedPick.getObjectBeingPicked().getClass().getSimpleName();
         selectedPickLabel.setText(labelText);
         if (selectedPick.isHoveringNotPointingAt())
         {
            setPickPointColliding(selectedPick.getClosestPointOnSurface());
         }
         else
         {
            setPickRayColliding(selectedPick.getDistanceToControllerPickPoint());
         }
      }
   }

   public void renderImGuiTunerWidgets()
   {
      pickPoseTransformTuner.renderTunerWithYawPitchRoll(0.001);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (getModelInstance() != null)
      {
         getModelInstance().getRenderables(renderables, pool);
         pickPoseSphere.getRenderables(renderables, pool);
         radialMenu.getRenderables(renderables, pool);

         if (pickRayGraphic != null)
            pickRayGraphic.getRenderables(renderables, pool);
         if (pickRayCollisionPointGraphic != null)
            pickRayCollisionPointGraphic.getRenderables(renderables, pool);
         if (aButtonLabel != null)
            aButtonLabel.getRenderables(renderables, pool);
         if (bButtonLabel != null)
            bButtonLabel.getRenderables(renderables, pool);
         if (selectedPickLabel != null)
            selectedPickLabel.getRenderables(renderables, pool);
         if (gripAmountLabel != null)
            gripAmountLabel.getRenderables(renderables, pool);
      }
   }

   /**
    * Updates the pick ray graphic. Only call this if you're not sumbitting a pick result.
    * Submitting pick results is better if possible, because they will get sorted and this
    * automatically called.
    */
   public void setPickRayColliding(double distance)
   {
      Point3D offset = new Point3D(distance / 2.0, 0.0, 0.0);
      ModelInstance pickRayBox = RDXModelBuilder.buildModelInstance(meshBuilder ->
         meshBuilder.addBox((float) distance, 0.001f, 0.001f, offset, new Color(Color.WHITE)), "box");
      pickRayGraphic = new RDXModelInstance(pickRayBox);
      pickRayGraphic.setPoseInWorldFrame(getPickPointPose());

      pickCollisionPoint.setToZero(pickPoseFrame.getReferenceFrame());
      pickCollisionPoint.setX(distance);
      pickCollisionPoint.changeFrame(ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(pickCollisionPoint, pickRayCollisionPointGraphic.transform);
   }

   /**
    * Updates the pick point graphic. Only call this if you're not sumbitting a pick result.
    * Submitting pick results is better if possible, because they will get sorted and this
    * automatically called.
    */
   public void setPickPointColliding(Point3DReadOnly closestPointOnSurface)
   {
      LibGDXTools.toLibGDX(closestPointOnSurface, pickRayCollisionPointGraphic.transform);
   }

   public RDXVRControllerModel getModel()
   {
      return model;
   }

   public void setModel(RDXVRControllerModel model)
   {
      this.model = model;
   }

   public RDXModelInstance getPickPoseSphere()
   {
      return pickPoseSphere;
   }

   public InputDigitalActionData getClickTriggerActionData()
   {
      return clickTriggerActionData;
   }

   public InputDigitalActionData getTriggerTouchedActionData()
   {
      return triggerTouchedActionData;
   }

   public InputAnalogActionData getTriggerActionData()
   {
      return triggerActionData;
   }

   public RDXVRDragData getTriggerDragData()
   {
      return triggerDragData;
   }

   public boolean getTriggerClickReleasedWithoutDrag()
   {
      return clickTriggerActionData.bChanged() && !clickTriggerActionData.bState() && triggerDragData.isClickValid();
   }

   public InputDigitalActionData getAButtonActionData()
   {
      return aButtonActionData;
   }

   public InputDigitalActionData getATouchedActionData()
   {
      return aTouchedActionData;
   }

   public InputDigitalActionData getBButtonActionData()
   {
      return bButtonActionData;
   }

   public InputDigitalActionData getBButtonDoubleClickActionData()
   {
      return bButtonDoubleClickActionData;
   }

   public InputDigitalActionData getBTouchedActionData()
   {
      return bTouchedActionData;
   }

   public InputDigitalActionData getJoystickPressActionData()
   {
      return joystickPressActionData;
   }

   public InputAnalogActionData getTouchpadActionData()
   {
      return touchpadActionData;
   }

   public InputDigitalActionData getTouchpadTouchedActionData()
   {
      return touchpadTouchedActionData;
   }

   public InputAnalogActionData getJoystickActionData()
   {
      return joystickActionData;
   }

   public boolean getJoystickIsCentered()
   {
      return joystickIsCentered;
   }

   public InputAnalogActionData getGripActionData()
   {
      return gripActionData;
   }

   public boolean getGripAsButtonDown()
   {
      return gripAsButtonDown;
   }

   public RDXVRDragData getGripDragData()
   {
      return gripDragData;
   }

   public ReferenceFrame getXForwardZUpControllerFrame()
   {
      return xForwardZUpControllerFrame;
   }

   public void getTransformZUpToWorld(Matrix4 transform)
   {
      xForwardZUpControllerFrame.getTransformToDesiredFrame(tempRigidBodyTransform, ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(tempRigidBodyTransform, transform);
   }

   public Pose3DReadOnly getXForwardZUpPose()
   {
      tempFramePose.setToZero(getXForwardZUpControllerFrame());
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      return tempFramePose;
   }

   public void runIfConnected(Consumer<RDXVRController> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }

   public void addPickResult(RDXVRPickResult pickResult)
   {
      pickResults.add(pickResult);
   }

   public RDXVRPickResult getSelectedPick()
   {
      return selectedPick;
   }

   /**
    * This is used to prevent UI components from responding to events or
    * getting initiated when other things are being interacted with already.
    * @param callersObject Pass your object in so we can exclude results that are already the caller's
    *                     object being dragged.
    * @return If anything else in the scene is being dragged.
    */
   public boolean anythingElseBeingDragged(Object callersObject)
   {
      boolean anythingElseBeingDragged = false;
      anythingElseBeingDragged |= gripDragData.isDraggingSomething() && gripDragData.getObjectBeingDragged() != callersObject;
      anythingElseBeingDragged |= triggerDragData.isDraggingSomething() && triggerDragData.getObjectBeingDragged() != callersObject;
      return anythingElseBeingDragged;
   }

   public FramePose3DReadOnly getPickPointPose()
   {
      return pickPoseFramePose;
   }

   public ReferenceFrame getPickPoseFrame()
   {
      return pickPoseFrame.getReferenceFrame();
   }

   public FrameLine3DReadOnly getPickRay()
   {
      return pickRay;
   }

   public void setAButtonText(String text)
   {
      aButtonLabel.setText(text);
   }

   public void setBButtonText(String text)
   {
      bButtonLabel.setText(text);
   }

   public RDXVRControllerRadialMenu getRadialMenu()
   {
      return radialMenu;
   }

   @Override
   public String toString()
   {
      return side.getSideNameFirstLetter() + ":" + getModel();
   }
}
