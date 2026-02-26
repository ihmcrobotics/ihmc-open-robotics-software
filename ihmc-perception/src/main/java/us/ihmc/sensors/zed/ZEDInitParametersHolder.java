package us.ihmc.sensors.zed;

import us.ihmc.zed.SL_InitParameters;

import static us.ihmc.zed.global.zed.*;

public class ZEDInitParametersHolder
{
   // ---------- Runtime-supplied values ----------
   private int inputType;
   private int resolution;
   private int cameraFps;
   private int cameraDeviceId;
   private int depthMode;
   private float depthMinimumDistance;
   private float depthMaximumDistance;

   // ---------- SDK Defaults (from setInitParametersOld) ----------
   private int cameraImageFlip = SL_FLIP_MODE_OFF;
   private boolean cameraDisableSelfCalib = false;
   private boolean enableImageEnhancement = true;
   private int depthStabilization = 100;
   private int coordinateUnit = SL_UNIT_METER;
   private int coordinateSystem = SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;
   private int sdkGpuId = -1;
   private int sdkVerbose = 0;
   private boolean sensorsRequired = true;
   private boolean enableRightSideMeasure = false;
   private float openTimeoutSec = 5.0f;
   private boolean asyncGrabCameraRecovery = false;

   // ---------- SDK defaults not explicitly set before ----------
   private boolean svoRealTimeMode = false;
   private float grabComputeCappingFps = 0.0f;
   private boolean enableImageValidityCheck = false;

   public ZEDInitParametersHolder()
   {
   }

   /**
    * Inject runtime-dependent parameters.
    * Keeps field defaults clean and separate.
    */
   public void configureFromSensor(int cameraID,
                                   ZEDModelData zedModel,
                                   int slInputType,
                                   int slDepthMode,
                                   int resolution,
                                   int fps)
   {
      this.cameraDeviceId = cameraID;
      this.inputType = slInputType;
      this.depthMode = slDepthMode;
      this.resolution = resolution;
      this.cameraFps = fps;

      this.depthMinimumDistance = zedModel.getMinimumDepthDistance();
      this.depthMaximumDistance = zedModel.getMaximumDepthDistance();
   }

   public void applyTo(SL_InitParameters params)
   {
      params.input_type(inputType);
      params.resolution(resolution);
      params.camera_fps(cameraFps);
      params.camera_device_id(cameraDeviceId);
      params.camera_image_flip(cameraImageFlip);
      params.camera_disable_self_calib(cameraDisableSelfCalib);
      params.enable_right_side_measure(enableRightSideMeasure);
      params.svo_real_time_mode(svoRealTimeMode);
      params.depth_mode(depthMode);
      params.depth_stabilization(depthStabilization);
      params.depth_minimum_distance(depthMinimumDistance);
      params.depth_maximum_distance(depthMaximumDistance);
      params.coordinate_unit(coordinateUnit);
      params.coordinate_system(coordinateSystem);
      params.sdk_gpu_id(sdkGpuId);
      params.sdk_verbose(sdkVerbose);
      params.sensors_required(sensorsRequired);
      params.enable_image_enhancement(enableImageEnhancement);
      params.open_timeout_sec(openTimeoutSec);
      params.async_grab_camera_recovery(asyncGrabCameraRecovery);
   }

   // -------- Getters and Setters --------

   public int getInputType() { return inputType; }
   public void setInputType(int inputType) { this.inputType = inputType; }

   public int getResolution() { return resolution; }
   public void setResolution(int resolution) { this.resolution = resolution; }

   public int getCameraFps() { return cameraFps; }
   public void setCameraFps(int cameraFps) { this.cameraFps = cameraFps; }

   public int getCameraDeviceId() { return cameraDeviceId; }
   public void setCameraDeviceId(int cameraDeviceId) { this.cameraDeviceId = cameraDeviceId; }

   public int getCameraImageFlip() { return cameraImageFlip; }
   public void setCameraImageFlip(int cameraImageFlip) { this.cameraImageFlip = cameraImageFlip; }

   public boolean isCameraDisableSelfCalib() { return cameraDisableSelfCalib; }
   public void setCameraDisableSelfCalib(boolean cameraDisableSelfCalib) { this.cameraDisableSelfCalib = cameraDisableSelfCalib; }

   public boolean isEnableRightSideMeasure() { return enableRightSideMeasure; }
   public void setEnableRightSideMeasure(boolean enableRightSideMeasure) { this.enableRightSideMeasure = enableRightSideMeasure; }

   public boolean isSvoRealTimeMode() { return svoRealTimeMode; }
   public void setSvoRealTimeMode(boolean svoRealTimeMode) { this.svoRealTimeMode = svoRealTimeMode; }

   public int getDepthMode() { return depthMode; }
   public void setDepthMode(int depthMode) { this.depthMode = depthMode; }

   public int getDepthStabilization() { return depthStabilization; }
   public void setDepthStabilization(int depthStabilization) { this.depthStabilization = depthStabilization; }

   public float getDepthMinimumDistance() { return depthMinimumDistance; }
   public void setDepthMinimumDistance(float depthMinimumDistance) { this.depthMinimumDistance = depthMinimumDistance; }

   public float getDepthMaximumDistance() { return depthMaximumDistance; }
   public void setDepthMaximumDistance(float depthMaximumDistance) { this.depthMaximumDistance = depthMaximumDistance; }

   public int getCoordinateUnit() { return coordinateUnit; }
   public void setCoordinateUnit(int coordinateUnit) { this.coordinateUnit = coordinateUnit; }

   public int getCoordinateSystem() { return coordinateSystem; }
   public void setCoordinateSystem(int coordinateSystem) { this.coordinateSystem = coordinateSystem; }

   public int getSdkGpuId() { return sdkGpuId; }
   public void setSdkGpuId(int sdkGpuId) { this.sdkGpuId = sdkGpuId; }

   public int getSdkVerbose() { return sdkVerbose; }
   public void setSdkVerbose(int sdkVerbose) { this.sdkVerbose = sdkVerbose; }

   public boolean isSensorsRequired() { return sensorsRequired; }
   public void setSensorsRequired(boolean sensorsRequired) { this.sensorsRequired = sensorsRequired; }

   public boolean isEnableImageEnhancement() { return enableImageEnhancement; }
   public void setEnableImageEnhancement(boolean enableImageEnhancement) { this.enableImageEnhancement = enableImageEnhancement; }

   public float getOpenTimeoutSec() { return openTimeoutSec; }
   public void setOpenTimeoutSec(float openTimeoutSec) { this.openTimeoutSec = openTimeoutSec; }

   public boolean isAsyncGrabCameraRecovery() { return asyncGrabCameraRecovery; }
   public void setAsyncGrabCameraRecovery(boolean asyncGrabCameraRecovery) { this.asyncGrabCameraRecovery = asyncGrabCameraRecovery; }

   public float getGrabComputeCappingFps() { return grabComputeCappingFps; }
   public void setGrabComputeCappingFps(float grabComputeCappingFps) { this.grabComputeCappingFps = grabComputeCappingFps; }

   public boolean isEnableImageValidityCheck() { return enableImageValidityCheck; }
   public void setEnableImageValidityCheck(boolean enableImageValidityCheck) { this.enableImageValidityCheck = enableImageValidityCheck; }
}
