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

import java.util.function.Supplier;

import static com.badlogic.gdx.math.Matrix4.*;

/**
 * A {@link Camera} implementation for one {@link RobotSide}
 * of a {@link GDXVRContext}. All properties  will be overwritten
 * on every call to {@link #update()} based on the tracked values
 * from the head mounted display.
 */
public class GDXVRCamera extends Camera
{
   private final RobotSide eye;
   private final Supplier<GDXVRDevice> headsetSupplier;
   private final Matrix4 eyeToHeadMatrix4 = new Matrix4();
   private final Matrix4 headToEyeMatrix4 = new Matrix4();
   private final HmdMatrix44 projectionHmdMatrix44 = HmdMatrix44.create();
   private final HmdMatrix34 eyeToHeadHmdMatrix34 = HmdMatrix34.create();
   private final Vector3 target = new Vector3();

   private final Vector3D euclidDirection = new Vector3D();
   private final Vector3D euclidUp = new Vector3D();

   public GDXVRCamera(RobotSide eye, Supplier<GDXVRDevice> headsetSupplier, int width, int height)
   {
      this.headsetSupplier = headsetSupplier;
      this.eye = eye;
      viewportWidth = width;
      viewportHeight = height;
   }

   @Override
   public void update()
   {
      update(true);
   }

   @Override
   public void update(boolean updateFrustum)
   {
      VRSystem.VRSystem_GetProjectionMatrix(eye.ordinal(), near, far, projectionHmdMatrix44);
      GDXTools.toGDX(projectionHmdMatrix44, projection);

      VRSystem.VRSystem_GetEyeToHeadTransform(eye.ordinal(), eyeToHeadHmdMatrix34);
      GDXTools.toGDX(eyeToHeadHmdMatrix34, eyeToHeadMatrix4);
      headToEyeMatrix4.set(eyeToHeadMatrix4).inv();

      GDXVRDevice headset = headsetSupplier.get();
      headset.getPose().changeFrame(ReferenceFrame.getWorldFrame());
      euclidDirection.set(Axis3D.Z);
      euclidDirection.negate(); // Z is forward in libGDX, contrary to OpenVR where it's backward
      headset.getPose().getOrientation().transform(euclidDirection);
      euclidUp.set(Axis3D.Y); // camera is rendered in Y up
      headset.getPose().getOrientation().transform(euclidUp);

      GDXTools.toGDX(headset.getPose().getPosition(), position);
//      position.add(eyeToHeadMatrix4.val[M03], eyeToHeadMatrix4.val[M13], eyeToHeadMatrix4.val[M23]);
      direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
      up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());

      target.set(position).add(direction);
      view.setToLookAt(position, target, up);

      combined.set(projection);
      Matrix4.mul(combined.val, headToEyeMatrix4.val);
      Matrix4.mul(combined.val, view.val);

      if (updateFrustum)
      {
         invProjectionView.set(combined);
         Matrix4.inv(invProjectionView.val);
         frustum.update(invProjectionView);
      }
   }
}
