package us.ihmc.gdx.vr;

import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.HmdMatrix44;
import org.lwjgl.openvr.VRSystem;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A {@link Camera} implementation for one {@link RobotSide}
 * of a {@link GDXVRContext}. All properties except {@link Camera#near},
 * {@link Camera#far} and {@link #offset} will be overwritten
 * on every call to {@link #update()} based on the tracked values
 * from the head mounted display. The {@link #offset}
 * vector allows you to position the camera in world space.
 *
 * @author badlogic
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
      Vector3 y = hmd.getUp(GDXVRSpace.World);
      Vector3 z = hmd.getDirection(GDXVRSpace.World);
      Vector3 p = hmd.getPosition(GDXVRSpace.World);

      view.idt();
      view.setToLookAt(p, tmp.set(p).add(z), y);

      position.set(p);
      direction.set(z);
      up.set(y);

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
