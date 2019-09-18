package us.ihmc.stateEstimation.head;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface AvatarHeadPoseEstimatorInterface
{
   /**
    * Packs the most recent estimate of the head pose.
    *
    * @param headTransform to be packed.
    */
   void getHeadTransform(RigidBodyTransform headTransform);

   /**
    * Initializes and resets the estimation. This method allows to calibrate the magnetic field direction in world and
    * to initialize the estimated pose with an initial guess. The call to this method is optional: by default the head
    * pose will start as identity and the magnetic field direction (north) will start as the positive x-axis in world.
    *
    * @param initialHeadTransform the initial guess for the head pose.
    * @param magneticFieldDirection the "north" direction in world frame.
    */
   void initialize(RigidBodyTransform initialHeadTransform, FrameVector3D magneticFieldDirection);

   /**
    * Call every tick to perform one estimation update (prediction and correction). Call
    * each tick with the newest measurements before calling this method. After the call to this method the newest head
    * pose estimate can be obtained via {@link #getHeadTransform(RigidBodyTransform)}.
    */
   void compute();

   YoVariableRegistry getRegistry();
}
