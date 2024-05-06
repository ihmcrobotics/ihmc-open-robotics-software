package us.ihmc.rdx.vr;

import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class RDXVRBaseStation extends RDXVRTrackedDevice
{
   public RDXVRBaseStation(ReferenceFrame vrPlayAreaYUpZBackFrame, int deviceIndex)
   {
      super(vrPlayAreaYUpZBackFrame);
      setDeviceIndex(deviceIndex);
      setConnected(true);
   }

   public void update(RDXVRTrackedDevicePose[] trackedDevicePoses)
   {
      setConnected(VRSystem.VRSystem_IsTrackedDeviceConnected(VR.k_unTrackedDeviceIndex_Hmd));

      super.update(trackedDevicePoses);
   }
}
