package us.ihmc.rdx.vr;

import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class GDXVRBaseStation extends GDXVRTrackedDevice
{
   public GDXVRBaseStation(ReferenceFrame vrPlayAreaYUpZBackFrame, int deviceIndex)
   {
      super(vrPlayAreaYUpZBackFrame);
      setDeviceIndex(deviceIndex);
      setConnected(true);
   }

   public void update(TrackedDevicePose.Buffer trackedDevicePoses)
   {
      setConnected(VRSystem.VRSystem_IsTrackedDeviceConnected(VR.k_unTrackedDeviceIndex_Hmd));

      super.update(trackedDevicePoses);
   }
}
