package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.HmdMatrix34;
import org.lwjgl.openvr.TrackedDevicePose;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXTools;

import java.nio.IntBuffer;

public abstract class GDXVRTrackedDevice
{
   private final ReferenceFrame vrPlayAreaYUpZBackFrame;
   private int deviceIndex;
   private boolean isConnected;

   private final IntBuffer errorCode = BufferUtils.newIntBuffer(1);
   private final FramePose3D pose = new FramePose3D();
   private final RotationMatrix tempRotationMatrix = new RotationMatrix();
   private ModelInstance modelInstance = null;

   public GDXVRTrackedDevice(ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      this.vrPlayAreaYUpZBackFrame = vrPlayAreaYUpZBackFrame;
   }

   protected void update(TrackedDevicePose.Buffer trackedDevicePoses)
   {
      if (isConnected)
      {
         TrackedDevicePose trackedDevicePose = trackedDevicePoses.get(deviceIndex);
         HmdMatrix34 openVRRigidBodyTransform = trackedDevicePose.mDeviceToAbsoluteTracking();
         pose.setReferenceFrame(vrPlayAreaYUpZBackFrame);
         GDXTools.toEuclid(openVRRigidBodyTransform, pose);
         pose.changeFrame(ReferenceFrame.getWorldFrame());

         if (modelInstance == null && isConnected)
         {
            String renderModelName = VRSystem.VRSystem_GetStringTrackedDeviceProperty(deviceIndex,
                                                                                      VR.ETrackedDeviceProperty_Prop_RenderModelName_String,
                                                                                      errorCode);
            Model model = GDXVRModelLoader.loadRenderModel(renderModelName);
            modelInstance = model != null ? new ModelInstance(model) : null;
         }
         if (modelInstance != null)
         {
            getGDXPoseInFrame(ReferenceFrame.getWorldFrame(), modelInstance.transform);
         }
      }
   }

   public void getGDXPoseInFrame(ReferenceFrame referenceFrame, Matrix4 gdxRigidBodyTransform)
   {
      pose.changeFrame(referenceFrame);
      pose.getOrientation().get(tempRotationMatrix);
      GDXTools.toGDX(tempRotationMatrix, gdxRigidBodyTransform);
      GDXTools.toGDX(pose.getPosition(), gdxRigidBodyTransform);
   }

   protected void setDeviceIndex(int deviceIndex)
   {
      this.deviceIndex = deviceIndex;
   }

   protected void setConnected(boolean isConnected)
   {
      this.isConnected = isConnected;
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public FramePose3D getPose()
   {
      return pose;
   }

   public boolean isConnected()
   {
      return isConnected;
   }

   public int getDeviceIndex()
   {
      return deviceIndex;
   }
}
