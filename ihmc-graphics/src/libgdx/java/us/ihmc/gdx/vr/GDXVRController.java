package us.ihmc.gdx.vr;

import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private final LongBuffer aButtonActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData aButtonActionData;
   private final LongBuffer bButtonActionHandle = BufferUtils.newLongBuffer(1);
   private InputDigitalActionData bButtonActionData;

   public GDXVRController(RobotSide side, ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      this.side = side;
   }

   public void initSystem()
   {
      inputOriginInfo = InputOriginInfo.create(1);
      VRInput.VRInput_GetInputSourceHandle("/user/hand/" + side.getLowerCaseName(), inputSourceHandle);

      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_clicktrigger", clickTriggerActionHandle);
      clickTriggerActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_abutton", aButtonActionHandle);
      aButtonActionData = InputDigitalActionData.create();
      VRInput.VRInput_GetActionHandle("/actions/main/in/" + side.getLowerCaseName() + "_bbutton", bButtonActionHandle);
      bButtonActionData = InputDigitalActionData.create();
   }

   public void update(TrackedDevicePose.Buffer trackedDevicePoses)
   {
      VRInput.VRInput_GetOriginTrackedDeviceInfo(inputSourceHandle.get(0), inputOriginInfo.get(0));
      setDeviceIndex(inputOriginInfo.trackedDeviceIndex());
      // VRSystem.VRSystem_IsTrackedDeviceConnected(leftHandDeviceIndex);
      setConnected(getDeviceIndex() != VR.k_unTrackedDeviceIndexInvalid);

      super.update(trackedDevicePoses);

      VRInput.VRInput_GetDigitalActionData(clickTriggerActionHandle.get(0), clickTriggerActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(aButtonActionHandle.get(0), aButtonActionData, VR.k_ulInvalidInputValueHandle);
      VRInput.VRInput_GetDigitalActionData(bButtonActionHandle.get(0), bButtonActionData, VR.k_ulInvalidInputValueHandle);
   }

   public InputDigitalActionData getClickTriggerActionData()
   {
      return clickTriggerActionData;
   }

   public InputDigitalActionData getAButtonActionData()
   {
      return aButtonActionData;
   }

   public InputDigitalActionData getBButtonActionData()
   {
      return bButtonActionData;
   }

   public void runIfConnected(Consumer<GDXVRController> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }
}
