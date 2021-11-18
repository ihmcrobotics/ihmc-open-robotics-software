package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDX3DSceneBasics;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVREye extends Camera
{
   private final GLFrameBuffer<Texture> frameBuffer;
   private final org.lwjgl.openvr.Texture openVRTexture;

   private final RobotSide side;
   private final GDXVRHeadset headset;
   private final Matrix4 eyeToHeadMatrix4 = new Matrix4();
   private final Matrix4 headToEyeMatrix4 = new Matrix4();
   private final HmdMatrix44 projectionHmdMatrix44 = HmdMatrix44.create();
   private final HmdMatrix34 eyeToHeadHmdMatrix34 = HmdMatrix34.create();
   private final Vector3 target = new Vector3();

   private final ReferenceFrame openVRFrame;
   private final RigidBodyTransform headsetToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame headsetFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("headsetFrame",
                                                                                                               ReferenceFrame.getWorldFrame(),
                                                                                                               headsetToWorldTransform);
   private final RigidBodyTransform eyeToHeadTransform = new RigidBodyTransform();
   private final ReferenceFrame eyeFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("eyeFrame",
                                                                                                           headsetFrame,
                                                                                                           eyeToHeadTransform);
   private final FramePose3D eyePose = new FramePose3D();

   private final Vector3D euclidDirection = new Vector3D();
   private final Vector3D euclidUp = new Vector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ModelInstance coordinateFrameInstance;

   public GDXVREye(RobotSide side,
                   GDXVRHeadset headset,
                   ReferenceFrame openVRFrame,
                   int width,
                   int height)
   {
      this.headset = headset;
      this.side = side;
      this.openVRFrame = openVRFrame;

      boolean hasDepth = true;
      boolean hasStencil = false;
      frameBuffer = new FrameBuffer(Pixmap.Format.RGBA8888, width, height, hasDepth, hasStencil);

      openVRTexture = org.lwjgl.openvr.Texture.create();
      openVRTexture.set(frameBuffer.getColorBufferTexture().getTextureObjectHandle(), VR.ETextureType_TextureType_OpenGL, VR.EColorSpace_ColorSpace_Gamma);

      viewportWidth = width;
      viewportHeight = height;
      near = 0.1f;
      far = 1000.0f;

      coordinateFrameInstance = GDXModelPrimitives.createCoordinateFrameInstance(0.3);
   }

   @Override
   public void update()
   {
      update(true);
   }

   @Override
   public void update(boolean updateFrustum)
   {
      headset.getPose().changeFrame(ReferenceFrame.getWorldFrame());
      headset.getPose().get(headsetToWorldTransform);
      headsetFrame.update();

      VRSystem.VRSystem_GetEyeToHeadTransform(side == RobotSide.LEFT ? VR.EVREye_Eye_Left : VR.EVREye_Eye_Right, eyeToHeadHmdMatrix34);
      GDXTools.toEuclid(eyeToHeadHmdMatrix34, eyeToHeadTransform);
      eyeFrame.update();

      eyePose.setToZero(eyeFrame);
      eyePose.changeFrame(ReferenceFrame.getWorldFrame());


      //      GDXTools.toGDX(eyeToHeadHmdMatrix34, eyeToHeadMatrix4);
      //      headToEyeMatrix4.set(eyeToHeadMatrix4).inv();

      //      headset.getPose().changeFrame(ReferenceFrame.getWorldFrame());
      //      euclidDirection.set(Axis3D.Z);
      //      euclidDirection.negate(); // Z is forward in libGDX, contrary to OpenVR where it's backward
      //      eyePose.getOrientation().transform(euclidDirection);
      //      euclidUp.set(Axis3D.Y); // camera is rendered in Y up
      //      eyePose.getOrientation().transform(euclidUp);
      //
      ////      position.set()
      //      GDXTools.toGDX(eyePose.getPosition(), position);
      ////      position.add(eyeToHeadMatrix4.val[M03], eyeToHeadMatrix4.val[M13], eyeToHeadMatrix4.val[M23]);
      ////      position.add(eyeToHeadMatrix4.val[M03], eyeToHeadMatrix4.val[M13], eyeToHeadMatrix4.val[M23]);
      //      direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
      //      up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());

      { // TODO: Copied from FocusBasedGDXCamera. What is going on here? Do we need an explicit libGDXFrame? or renderFrame?
         euclidDirection.set(Axis3D.Z);
         euclidDirection.negate();
         eyePose.getOrientation().transform(euclidDirection);
         euclidUp.set(Axis3D.Y); // camera is rendered in Y up
         eyePose.getOrientation().transform(euclidUp);

         GDXTools.toGDX(eyePose.getPosition(), position);
         direction.set(euclidDirection.getX32(), euclidDirection.getY32(), euclidDirection.getZ32());
         up.set(euclidUp.getX32(), euclidUp.getY32(), euclidUp.getZ32());
      }

//      GDXTools.toGDX(eyePose.getPosition(), position);
//      GDXTools.toGDX(eyePose.getPosition(), direction);

      VRSystem.VRSystem_GetProjectionMatrix(side.ordinal(), Math.abs(near), Math.abs(far), projectionHmdMatrix44);
      GDXTools.toGDX(projectionHmdMatrix44, projection);

      //      target.set(position).add(direction);
      //      view.setToLookAt(position, target, up);
      //
      //      combined.set(projection);
      //      Matrix4.mul(combined.val, headToEyeMatrix4.val);
      //      Matrix4.mul(combined.val, view.val);

      view.setToLookAt(position, target.set(position).add(direction), up);
      combined.set(projection);
      Matrix4.mul(combined.val, view.val);

      if (updateFrustum)
      {
         invProjectionView.set(combined);
         Matrix4.inv(invProjectionView.val);
         frustum.update(invProjectionView);
      }

      eyePose.get(tempTransform);
      GDXTools.toGDX(tempTransform, coordinateFrameInstance.transform);
   }

   public void render(GDX3DSceneBasics sceneBasics)
   {
      update();
      frameBuffer.begin();
      GL41.glViewport(0, 0, (int) viewportWidth, (int) viewportHeight);
      GDX3DSceneTools.glClearGray();
      sceneBasics.renderToCamera(this);
      frameBuffer.end();
   }

   public FramePose3D getEyePose()
   {
      return eyePose;
   }

   public GLFrameBuffer<Texture> getFrameBuffer()
   {
      return frameBuffer;
   }

   public org.lwjgl.openvr.Texture getOpenVRTexture()
   {
      return openVRTexture;
   }

   public ModelInstance getCoordinateFrameInstance()
   {
      return coordinateFrameInstance;
   }
}
