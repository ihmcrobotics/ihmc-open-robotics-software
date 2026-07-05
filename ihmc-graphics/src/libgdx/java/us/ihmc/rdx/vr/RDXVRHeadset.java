package us.ihmc.rdx.vr;

import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRInput;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

import java.nio.LongBuffer;
import java.util.function.Consumer;

public class RDXVRHeadset extends RDXVRTrackedDevice
{
   private final LongBuffer inputSourceHandle = BufferUtils.newLongBuffer(1);
   private final RigidBodyTransform headsetToWorldTransform = new RigidBodyTransform();
   private static final RigidBodyTransformReadOnly headsetXRightZDownToXForwardZUp = new RigidBodyTransform(
         new YawPitchRoll(         // For this transformation, we start with IHMC ZUp with index forward and thumb up
            Math.toRadians(90.0),  // rotating around thumb, index goes forward to left
            Math.toRadians(90.0),  // rotating about middle finger, index goes left to down
            Math.toRadians(0.0)    // no rotation about index finger
         ),
         new Point3D()
   );
   private final ReferenceFrame xForwardZUpHeadsetFrame;

   // Camera-only POV offset: a frame between the headset device frame and the eyes (see RDXVREye), so a teleop app can
   // pitch/raise the rendered viewpoint WITHOUT moving the controllers/trackers (those stay on the play area). Identity
   // by default, so non-teleop VR apps are unaffected. In the device frame the axes are X-right, Y-up, Z-back.
   private final RigidBodyTransform cameraPovOffsetTransform = new RigidBodyTransform();
   private final ReferenceFrame cameraPovOffsetFrame;

   public RDXVRHeadset(ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      setDeviceIndex(VR.k_unTrackedDeviceIndex_Hmd);

      cameraPovOffsetFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("cameraPovOffsetFrame",
                                                                                             getDeviceYUpZBackFrame(),
                                                                                             cameraPovOffsetTransform);
      xForwardZUpHeadsetFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("xForwardZUpHeadsetFrame",
                                                                                getDeviceYUpZBackFrame(),
                                                                                headsetXRightZDownToXForwardZUp);
   }

   /**
    * Camera-only POV offset (does NOT move controllers/trackers). {@code pitchRad} tilts the view (forward-lean
    * positive -> look down); {@code heightMeters} raises (+) / lowers (-) the viewpoint. Applied about the head, in the
    * device frame (X-right, Y-up, Z-back): height is along +Y, view pitch is a rotation about the +X (right) axis.
    * NOTE: signs to be confirmed in VR -- flip pitchRad/heightMeters here if the POV moves the wrong way.
    */
   public void setCameraPovPitchAndHeight(double pitchRad, double heightMeters)
   {
      cameraPovOffsetTransform.setIdentity();
      cameraPovOffsetTransform.getTranslation().set(0.0, heightMeters, 0.0);
      cameraPovOffsetTransform.getRotation().setYawPitchRoll(0.0, 0.0, -pitchRad); // roll axis == device +X == view pitch
   }

   public ReferenceFrame getCameraPovOffsetFrame()
   {
      return cameraPovOffsetFrame;
   }

   public void initSystem()
   {
      VRInput.VRInput_GetInputSourceHandle("/user/head", inputSourceHandle);
   }

   public void update(RDXVRTrackedDevicePose[] trackedDevicePoses)
   {
      setConnected(VRSystem.VRSystem_IsTrackedDeviceConnected(VR.k_unTrackedDeviceIndex_Hmd));

      super.update(trackedDevicePoses);
   }

   public void runIfConnected(Consumer<RDXVRHeadset> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }

   public ReferenceFrame getXForwardZUpHeadsetFrame()
   {
      return xForwardZUpHeadsetFrame;
   }
}
