package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.imgui.ImGuiRigidBodyTransformTuner;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.nio.LongBuffer;
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
   private final RobotSide side;

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
   private final LongBuffer gripActionHandle = BufferUtils.newLongBuffer(1);
   private InputAnalogActionData gripActionData;
   private final SideDependentList<FrameLine3D> vrPickRays = new SideDependentList<>(() -> new FrameLine3D());
   private final SideDependentList<FramePose3D> vrPickRayPose = new SideDependentList<>(() -> new FramePose3D());

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
   private final ModifiableReferenceFrame pickPoseFrame;
   private final ImGuiRigidBodyTransformTuner pickPoseTransformTuner;
   private RDXModelInstance pickPoseSphere;

   public RDXVRController(RobotSide side, ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      this.side = side;

      xForwardZUpControllerFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(side.getLowerCaseName() + "_xForwardZUpControllerFrame",
                                                                                getDeviceYUpZBackFrame(),
                                                                                controllerYBackZLeftXRightToXForwardZUp);
      pickPoseFrame = new ModifiableReferenceFrame(xForwardZUpControllerFrame);
      pickPoseFrame.getTransformToParent().getTranslation().setX(0.029);
      pickPoseFrame.getTransformToParent().getTranslation().setY(side.negateIfLeftSide(0.020));
      pickPoseFrame.getTransformToParent().getTranslation().setZ(-0.017);
      pickPoseFrame.getReferenceFrame().update();
      pickPoseTransformTuner = new ImGuiRigidBodyTransformTuner(pickPoseFrame.getTransformToParent());
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

   public void update(TrackedDevicePose.Buffer trackedDevicePoses)
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
         }

         pickPoseFrame.getReferenceFrame().update();
         pickPoseFramePose.setToZero(pickPoseFrame.getReferenceFrame());
         pickPoseFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         pickPoseSphere.setPoseInWorldFrame(pickPoseFramePose);
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
   }

   public void renderImGuiTunerWidgets()
   {
      pickPoseTransformTuner.renderTunerWithYawPitchRoll(0.001);
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

   public InputAnalogActionData getGripActionData()
   {
      return gripActionData;
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

   public FramePose3DReadOnly getPickPointPose()
   {
      return pickPoseFramePose;
   }

   public ReferenceFrame getPickPoseFrame()
   {
      return pickPoseFrame.getReferenceFrame();
   }

   public FrameLine3D getPickRay(RobotSide side)
   {
      vrPickRays.get(side).setToZero(getXForwardZUpControllerFrame());
      vrPickRays.get(side).getDirection().set(Axis3D.X);
      vrPickRays.get(side).changeFrame(ReferenceFrame.getWorldFrame());

      vrPickRayPose.get(side).setToZero(getXForwardZUpControllerFrame());
      vrPickRayPose.get(side).changeFrame(ReferenceFrame.getWorldFrame());

      return vrPickRays.get(side);
   }
}
