package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.VR;
import org.lwjgl.openvr.VRSystem;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;

import java.nio.IntBuffer;

public abstract class RDXVRTrackedDevice
{
   private int deviceIndex;
   private boolean isConnected;
   private final IntBuffer errorCode = BufferUtils.newIntBuffer(1);
   private final RigidBodyTransform deviceToPlayAreaTransform = new RigidBodyTransform();
   private final ReferenceFrame deviceYUpZBackFrame;
   private final RigidBodyTransform tempOpenVRToWorldTransform = new RigidBodyTransform();
   private ModelInstance modelInstance = null;
   private long lastPollTimeNanos;
   private final Vector3D trackedLinearVelocity = new Vector3D();
   private final Vector3D trackedAngularVelocity = new Vector3D();

   public RDXVRTrackedDevice(ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      deviceYUpZBackFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("device" + deviceIndex + "YUpZBackFrame",
                                                                                            vrPlayAreaYUpZBackFrame,
                                                                                            deviceToPlayAreaTransform);
   }

   protected void update(TrackedDevicePoseParsed[] trackedDevicePoses)
   {
      if (isConnected)
      {
         lastPollTimeNanos = trackedDevicePoses[deviceIndex].timestamp;

         // We do this stuff to try and safely catch exceptions
         deviceToPlayAreaTransform.set(trackedDevicePoses[deviceIndex].mDeviceToAbsoluteTracking);
         trackedLinearVelocity.set(trackedDevicePoses[deviceIndex].vVelocity);
         trackedAngularVelocity.set(trackedDevicePoses[deviceIndex].vAngularVelocity);

         boolean matrixInvalid = deviceToPlayAreaTransform.containsNaN();
         if (!matrixInvalid)
         {
            try
            {
               deviceToPlayAreaTransform.getRotation().normalize();
            }
            catch (NotARotationMatrixException notARotationMatrixException)
            {
               matrixInvalid = true;
               LogTools.error(deviceToPlayAreaTransform.getRotation().containsNaN() ?
                       "Not a rotation matrix: Normalization failed. Result contains NaN."
                       : notARotationMatrixException.getMessage());
            }
         }

         if (matrixInvalid)
         {
            isConnected = false;
         }
         else
         {
            deviceYUpZBackFrame.update();

            if (modelInstance == null)
            {
               String renderModelName = getModelName();
               Model model = new Model();
               if (renderModelName.contains("controller"))
               {
                  if (renderModelName.contains("focus3")) // vive focus 3 controller render models are not supported in open vr
                  {
                     String modelFile = "vr/controllers/vive_focus3/";
                     if (renderModelName.contains("left"))
                        modelFile += "Focus3_controller_left.g3dj";
                     else if (renderModelName.contains("right"))
                        modelFile += "Focus3_controller_right.g3dj";
                     model = RDXModelLoader.load(modelFile);
                  }
                  else if (renderModelName.contains("index"))
                  {
                     String modelFile = "vr/controllers/index/";
                     if (renderModelName.contains("left"))
                        modelFile += "valve_controller_knu_1_0_left.g3dj";
                     else if (renderModelName.contains("right"))
                        modelFile += "valve_controller_knu_1_0_right.g3dj";
                     model = RDXModelLoader.load(modelFile);
                  }
               }
               else
                  model = RDXVRModelLoader.loadRenderModel(renderModelName);
               modelInstance = model != null ? new ModelInstance(model) : null;
            }

            deviceYUpZBackFrame.getTransformToDesiredFrame(tempOpenVRToWorldTransform, ReferenceFrame.getWorldFrame());
            LibGDXTools.toLibGDX(tempOpenVRToWorldTransform, modelInstance.transform);
//            deviceToPlayAreaTransform.transform(trackedAngularVelocity);
            deviceYUpZBackFrame.getParent().getTransformToRoot().transform(trackedLinearVelocity);
            deviceYUpZBackFrame.getParent().getTransformToRoot().transform(trackedAngularVelocity);
         }
      }
   }

   public long getLastPollTimeNanos()
   {
      return lastPollTimeNanos;
   }

   public String getModelName()
   {
      return VRSystem.VRSystem_GetStringTrackedDeviceProperty(deviceIndex, VR.ETrackedDeviceProperty_Prop_RenderModelName_String, errorCode);
   }

   public ReferenceFrame getDeviceYUpZBackFrame()
   {
      return deviceYUpZBackFrame;
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

   public boolean isConnected()
   {
      return isConnected;
   }

   public int getDeviceIndex()
   {
      return deviceIndex;
   }

   public Vector3D getLinearVelocity()
   {
      return trackedLinearVelocity;
   }

   public Vector3D getAngularVelocity()
   {
      return trackedAngularVelocity;
   }
}
