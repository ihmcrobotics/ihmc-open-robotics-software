package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import org.lwjgl.opengl.GL41;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVREye extends Camera
{
   private final GLFrameBuffer<Texture> frameBuffer;
   private final org.lwjgl.openvr.Texture openVRTexture;

   private final RobotSide side;
   private final HmdMatrix44 projectionHmdMatrix44 = HmdMatrix44.create();
   private final HmdMatrix34 eyeToHeadHmdMatrix34 = HmdMatrix34.create();
   private final Vector3 target = new Vector3();

   private final RigidBodyTransform openVREyeToHeadTransform = new RigidBodyTransform();
   private final ReferenceFrame eyeXRightZBackFrame;
   private final RigidBodyTransformReadOnly xForwardZUpToXRightZBackTransform = new RigidBodyTransform(
         new YawPitchRoll(
            Math.toRadians(90.0),
            Math.toRadians(90.0),
            Math.toRadians(0.0)
         ),
         new Point3D()
   );
   private final ReferenceFrame eyeXForwardZUpFrame;
   private final FramePose3D eyeFramePose = new FramePose3D();
   private final Vector3D euclidDirection = new Vector3D();
   private final Vector3D euclidUp = new Vector3D();

   public RDXVREye(RobotSide side, RDXVRHeadset headset, int width, int height)
   {
      this.side = side;

      eyeXRightZBackFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(side.getLowerCaseName() + "EyeXRightZBackFrame",
                                                                                            headset.getDeviceYUpZBackFrame(),
                                                                                            openVREyeToHeadTransform);
      eyeXForwardZUpFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(side.getLowerCaseName() + "EyeXForwardZUpFrame",
                                                                                              eyeXRightZBackFrame,
                                                                                              xForwardZUpToXRightZBackTransform);

      boolean hasDepth = true;
      boolean hasStencil = false;
      frameBuffer = new FrameBuffer(Pixmap.Format.RGBA8888, width, height, hasDepth, hasStencil);

      openVRTexture = org.lwjgl.openvr.Texture.create();
      openVRTexture.set(frameBuffer.getColorBufferTexture().getTextureObjectHandle(), VR.ETextureType_TextureType_OpenGL, VR.EColorSpace_ColorSpace_Gamma);

      viewportWidth = width;
      viewportHeight = height;
      near = 0.1f;
      far = 1000.0f;
   }

   @Override
   public void update()
   {
      update(true);
   }

   @Override
   public void update(boolean updateFrustum)
   {
      VRSystem.VRSystem_GetEyeToHeadTransform(side == RobotSide.LEFT ? VR.EVREye_Eye_Left : VR.EVREye_Eye_Right, eyeToHeadHmdMatrix34);
      LibGDXTools.toEuclid(eyeToHeadHmdMatrix34, openVREyeToHeadTransform);
      eyeXRightZBackFrame.update();

      eyeFramePose.setToZero(eyeXForwardZUpFrame);
      eyeFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      euclidDirection.set(Axis3D.X);
      eyeFramePose.getOrientation().transform(euclidDirection);
      euclidUp.set(Axis3D.Z);
      eyeFramePose.getOrientation().transform(euclidUp);

      LibGDXTools.toLibGDX(eyeFramePose.getPosition(), position);
      direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
      up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());

      VRSystem.VRSystem_GetProjectionMatrix(side.ordinal(), Math.abs(near), Math.abs(far), projectionHmdMatrix44);
      LibGDXTools.toLibGDX(projectionHmdMatrix44, projection);

      view.setToLookAt(position, target.set(position).add(direction), up);
      combined.set(projection);
      Matrix4.mul(combined.val, view.val);

      if (updateFrustum)
      {
         invProjectionView.set(combined);
         Matrix4.inv(invProjectionView.val);
         frustum.update(invProjectionView);
      }
   }

   public void render(RDX3DScene scene)
   {
      update();
      frameBuffer.begin();
      GL41.glViewport(0, 0, (int) viewportWidth, (int) viewportHeight);
      RDX3DSceneTools.glClearGray();
      scene.renderToCamera(this);
      frameBuffer.end();
   }

   public RobotSide getSide()
   {
      return side;
   }

   public ReferenceFrame getEyeXForwardZUpFrame()
   {
      return eyeXForwardZUpFrame;
   }

   public ReferenceFrame getEyeXRightZBackFrame()
   {
      return eyeXRightZBackFrame;
   }

   public GLFrameBuffer<Texture> getFrameBuffer()
   {
      return frameBuffer;
   }

   public org.lwjgl.openvr.Texture getOpenVRTexture()
   {
      return openVRTexture;
   }
}
