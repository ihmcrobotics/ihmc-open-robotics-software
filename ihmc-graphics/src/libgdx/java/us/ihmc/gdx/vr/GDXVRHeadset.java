package us.ihmc.gdx.vr;

import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.nio.LongBuffer;
import java.util.function.Consumer;

public class GDXVRHeadset extends GDXVRTrackedDevice
{
   private final LongBuffer inputSourceHandle = BufferUtils.newLongBuffer(1);

   public GDXVRHeadset(ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      setDeviceIndex(VR.k_unTrackedDeviceIndex_Hmd);
   }

   public void initSystem()
   {
      VRInput.VRInput_GetInputSourceHandle("/user/head", inputSourceHandle);
   }

   public void update(TrackedDevicePose.Buffer trackedDevicePoses)
   {
      setConnected(VRSystem.VRSystem_IsTrackedDeviceConnected(VR.k_unTrackedDeviceIndex_Hmd));

      super.update(trackedDevicePoses);
   }

   public void runIfConnected(Consumer<GDXVRHeadset> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }
}
