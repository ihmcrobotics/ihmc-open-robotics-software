package us.ihmc.rdx.vr;

import com.badlogic.gdx.math.Matrix4;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.tools.LibGDXTools;

import java.util.function.Consumer;

public class RDXVRTracker extends RDXVRTrackedDevice
{
   private static final RigidBodyTransformReadOnly trackerYBackZLeftXRightToXForwardZUp = new RigidBodyTransform(
         new YawPitchRoll(          // For this transformation, we start with IHMC ZUp with index forward and thumb up
                                    Math.toRadians(0.0),
                                    Math.toRadians(90.0),
                                    Math.toRadians(0.0)
         ),
         new Point3D()
   );
   private final ReferenceFrame xForwardZUpTrackerFrame;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempRigidBodyTransform = new RigidBodyTransform();

   public RDXVRTracker(ReferenceFrame vrPlayAreaYUpZBackFrame, int deviceIndex)
   {
      super(vrPlayAreaYUpZBackFrame);
      setDeviceIndex(deviceIndex);
      setConnected(true);

      xForwardZUpTrackerFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("xForwardZUpTrackerFrame",
                                                                                getDeviceYUpZBackFrame(),
                                                                                trackerYBackZLeftXRightToXForwardZUp);
   }

   public void update(TrackedDevicePose.Buffer trackedDevicePoses)
   {
      setConnected(getDeviceIndex() != VR.k_unTrackedDeviceIndexInvalid);

      xForwardZUpTrackerFrame.update();
      super.update(trackedDevicePoses);
   }

   public void runIfConnected(Consumer<us.ihmc.rdx.vr.RDXVRTracker> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }

   public void getTransformZUpToWorld(Matrix4 transform)
   {
      xForwardZUpTrackerFrame.getTransformToDesiredFrame(tempRigidBodyTransform, ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(tempRigidBodyTransform, transform);
   }

   public Pose3DReadOnly getXForwardZUpPose()
   {
      tempFramePose.setToZero(getXForwardZUpTrackerFrame());
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      return tempFramePose;
   }

   public ReferenceFrame getXForwardZUpTrackerFrame()
   {
      return xForwardZUpTrackerFrame;
   }
}