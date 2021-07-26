package us.ihmc.gdx.vr;

import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;
import org.lwjgl.openvr.VRSystem;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A {@link Camera} implementation for one {@link RobotSide}
 * of a {@link GDXVRContext}. All properties  will be overwritten
 * on every call to {@link #update()} based on the tracked values
 * from the head mounted display.
 */
public class GDXVRCamera extends Camera
{
   private final GDXVRContext context;
   private final RobotSide eye;
   private final Matrix4 eyeSpace = new Matrix4();
   private final Matrix4 invEyeSpace = new Matrix4();
   private final HmdMatrix44 projectionMat = HmdMatrix44.create();
   private final HmdMatrix34 eyeMat = HmdMatrix34.create();
   private final Vector3 tmp = new Vector3();

   private final Vector3D euclidDirection = new Vector3D();
   private final Vector3D euclidUp = new Vector3D();

   public GDXVRCamera(GDXVRContext context, RobotSide eye)
   {
      this.context = context;
      this.eye = eye;
   }

   @Override
   public void update()
   {
      update(true);
   }

   @Override
   public void update(boolean updateFrustum)
   {
      // get the projection matrix from the HDM
      VRSystem.VRSystem_GetProjectionMatrix(eye.ordinal(), near, far, projectionMat);
      GDXTools.toGDX(projectionMat, projection);

      // get the eye space matrix from the HDM
      VRSystem.VRSystem_GetEyeToHeadTransform(eye.ordinal(), eyeMat);
      GDXTools.toGDX(eyeMat, eyeSpace);
      invEyeSpace.set(eyeSpace).inv();

      // get the pose matrix from the HDM
      GDXVRDevice hmd = context.getDeviceByType(GDXVRDeviceType.HeadMountedDisplay);
      hmd.getPose().changeFrame(ReferenceFrame.getWorldFrame());
      euclidDirection.set(Axis3D.Z);
      euclidDirection.negate(); // Z is forward in libGDX, contrary to OpenVR where it's backward
      hmd.getPose().getOrientation().transform(euclidDirection);
      euclidUp.set(Axis3D.Y); // camera is rendered in Y up
      hmd.getPose().getOrientation().transform(euclidUp);

      position.set(hmd.getPose().getPosition().getX32(), hmd.getPose().getPosition().getY32(), hmd.getPose().getPosition().getZ32());
      direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
      up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());

      view.setToLookAt(position, tmp.set(position).add(direction), up);

      combined.set(projection);
      Matrix4.mul(combined.val, invEyeSpace.val);
      Matrix4.mul(combined.val, view.val);

      if (updateFrustum)
      {
         invProjectionView.set(combined);
         Matrix4.inv(invProjectionView.val);
         frustum.update(invProjectionView);
      }
   }
}
