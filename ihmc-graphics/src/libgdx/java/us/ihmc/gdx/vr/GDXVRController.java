package us.ihmc.gdx.vr;

import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;

import java.nio.LongBuffer;
import java.util.function.Consumer;

public class GDXVRController extends GDXVRTrackedDevice
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

   private final RigidBodyTransform controllerToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame xRightZDownControllerFrame;
   private static final RigidBodyTransformReadOnly controllerXRightZDownToXForwardZUp = new RigidBodyTransform(
      new YawPitchRoll(          // For this transformation, we start with IHMC ZUp with index forward and thumb up
         Math.toRadians(90.0),  // rotating around thumb, index goes forward to right
         Math.toRadians(135.0),    // no rotation about middle finger
         Math.toRadians(0.0)   // rotating about index finger, thumb goes up to toward you then down
      ),
      new Point3D()
   );
   private final ReferenceFrame xForwardZUpControllerFrame;

   public GDXVRController(RobotSide side, ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      this.side = side;

      xRightZDownControllerFrame
            = ReferenceFrameTools.constructFrameWithChangingTransformToParent(side.getLowerCaseName() + "_xRightZDownControllerFrame",
                                                                              ReferenceFrame.getWorldFrame(),
                                                                              controllerToWorldTransform);
      xForwardZUpControllerFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(side.getLowerCaseName() + "_xForwardZUpControllerFrame",
                                                                                xRightZDownControllerFrame,
                                                                                controllerXRightZDownToXForwardZUp);
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

      getPose().get(controllerToWorldTransform);
      xRightZDownControllerFrame.update();

      VRInput.VRInput_GetDigitalActionData(clickTriggerActionHandle.get(0), clickTriggerActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(triggerTouchedActionHandle.get(0), triggerTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(triggerActionHandle.get(0), triggerActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(aButtonActionHandle.get(0), aButtonActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(aTouchedActionHandle.get(0), aTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(bButtonActionHandle.get(0), bButtonActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(bTouchedActionHandle.get(0), bTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(joystickPressActionHandle.get(0), joystickPressActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(touchpadActionHandle.get(0), touchpadActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(touchpadTouchedActionHandle.get(0), touchpadTouchedActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(joystickActionHandle.get(0), joystickActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetAnalogActionData(gripActionHandle.get(0), gripActionData, VR.k_ulInvalidInputValueHandle);
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

   public void runIfConnected(Consumer<GDXVRController> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }
}
