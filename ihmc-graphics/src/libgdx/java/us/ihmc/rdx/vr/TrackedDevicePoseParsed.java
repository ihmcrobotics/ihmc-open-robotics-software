package us.ihmc.rdx.vr;

import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdVector3;
import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VR;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.LibGDXTools;

public class TrackedDevicePoseParsed
{
   public final long timestamp;
   public final RigidBodyTransform mDeviceToAbsoluteTracking = new RigidBodyTransform();
   public final Vector3D vVelocity = new Vector3D();
   public final Vector3D vAngularVelocity = new Vector3D();

   public TrackedDevicePoseParsed(long timestamp, TrackedDevicePose.Buffer trackedDevicePoses, int deviceIndex)
   {
      this.timestamp = timestamp;
      if (deviceIndex == VR.k_unTrackedDeviceIndexInvalid)
         return;

      LibGDXTools.toEuclidUnsafe(trackedDevicePoses.get(deviceIndex).mDeviceToAbsoluteTracking(), mDeviceToAbsoluteTracking);
      LibGDXTools.toEuclid(trackedDevicePoses.get(deviceIndex).vVelocity(), vVelocity);
      LibGDXTools.toEuclid(trackedDevicePoses.get(deviceIndex).vAngularVelocity(), vAngularVelocity);
   }
}
