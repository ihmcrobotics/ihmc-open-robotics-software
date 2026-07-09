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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
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
   // TRANSLATE the rendered viewpoint WITHOUT moving the controllers/trackers (those stay on the play area). Identity
   // by default, so non-teleop VR apps are unaffected. In the device frame the axes are X-right, Y-up, Z-back.
   private final RigidBodyTransform cameraPovOffsetTransform = new RigidBodyTransform();
   private final ReferenceFrame cameraPovOffsetFrame;
   private final Vector3D cameraPovLocalTranslation = new Vector3D();

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
    * Camera-only POV position offset (does NOT move controllers/trackers, does NOT rotate the view -- the operator keeps
    * their own head pitch/roll, and yaw still follows the play-area teleport). {@code worldOffset} is the desired
    * displacement of the rendered viewpoint expressed in WORLD, so e.g. a straight-down offset always lowers the view in
    * world regardless of where the operator is looking. It is re-expressed here in the device-local frame (X-right,
    * Y-up, Z-back) using the current headset orientation so the camera origin moves by exactly {@code worldOffset}.
    */
   public void setCameraPovWorldPositionOffset(Vector3DReadOnly worldOffset)
   {
      cameraPovLocalTranslation.set(worldOffset);
      getDeviceYUpZBackFrame().getTransformToWorldFrame().getRotation().inverseTransform(cameraPovLocalTranslation);
      cameraPovOffsetTransform.setIdentity();
      cameraPovOffsetTransform.getTranslation().set(cameraPovLocalTranslation);
   }

   /** Reset the camera POV offset to identity (no viewpoint displacement); call when leaving teleop. */
   public void clearCameraPovOffset()
   {
      cameraPovOffsetTransform.setIdentity();
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
