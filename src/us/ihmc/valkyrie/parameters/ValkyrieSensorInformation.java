package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;

public class ValkyrieSensorInformation implements DRCRobotSensorInformation
{

   public static final String[] forceSensorNames;
   static
   {
      if (ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS)
         forceSensorNames = new String[]{ "LeftAnkle", "RightAnkle", "LeftForearmSupinator", "RightForearmSupinator" };
      else
         forceSensorNames = new String[]{ "LeftAnkle", "RightAnkle"};
   }
   
   private static final SideDependentList<String> tekscanSensorNames = new SideDependentList<String>("/v1/LeftLegHermes_Offset", "/v1/RightLegHermes_Offset");
   private static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("LeftAnkle", "RightAnkle");
   private static final SideDependentList<String> urdfFeetForceSensorNames = new SideDependentList<>("/v1/LeftLeg6Axis_Offset", "/v1/RightLeg6Axis_Offset");

   public static final boolean USE_JSC_FOOT_MASS_TARING = false;

   public static final SideDependentList<RigidBodyTransform> transformFromSixAxisMeasurementToAnkleZUpFrames = new SideDependentList<>();
   static
   {     
      RigidBodyTransform translateForwardAndDownOnFoot = new RigidBodyTransform();
      translateForwardAndDownOnFoot.setTranslation(new Vector3d(0.02150, 0.0, -0.058547));  //from Will's CAD measurement
      
      RigidBodyTransform rotYBy7dot5 = new RigidBodyTransform();
      rotYBy7dot5.rotY(-Math.PI/24.0);
      
      RigidBodyTransform rotXByPi = new RigidBodyTransform();
      rotXByPi.rotX(Math.PI);
      
      RigidBodyTransform rotateZ60Degrees = new RigidBodyTransform();
      rotateZ60Degrees.rotZ(-Math.PI/3.0);
      
      RigidBodyTransform leftTransform = new RigidBodyTransform();
      leftTransform.multiply(translateForwardAndDownOnFoot);
      leftTransform.multiply(rotYBy7dot5);
      leftTransform.multiply(rotateZ60Degrees);
      leftTransform.multiply(rotXByPi);

      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.LEFT, leftTransform);
      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.RIGHT, new RigidBodyTransform(leftTransform));
   }

   /**
    * PointCloud Parameters
    */
   private final DRCRobotPointCloudParameters[] pointCloudParamaters = new DRCRobotPointCloudParameters[1];
   public static final int IBEO_ID = 0;
   private static final String ibeoSensorName = "/v1/Ibeo_sensor";
   private static final String ibeoTopic = "/ibeo/points";
   
   private static final SideDependentList<String> wristForceSensorNames = new SideDependentList<String>("LeftForearmSupinator", "RightForearmSupinator");
   
   private static int forheadCameraId = 0;
   private static int leftHazardCameraId = 1;
   private static int rightHazardCameraId = 2;
   private static int primaryCameraId = forheadCameraId;
   
   private static final String headLinkName = "/v1/Head";
   private final DRCRobotCameraParameters[] cameraParamaters = new DRCRobotCameraParameters[3];
   
   private static final String forheadCameraName = "/v1/HeadWebcam___default__";
   private static final String forheadCameraTopic = "/forhead/image_raw/compressed";
   
   private static final String leftStereoCameraName = "/v1/LeftHazardCamera___default__";
   private static final String leftCameraTopic = "/v1/LeftHazardCamera/compressed";
   
   private static final String rightStereoCameraName ="/v1/RightHazardCamera___default__";
   private static final String rightCameraTopic = "/v1/rightHazardCamera/compressed";
   
   
   private static final String rightTrunkIMUSensor = "v1Trunk_RightIMU";
   private static final String leftTrunkIMUSensor = "v1Trunk_LeftIMU";
   private static final String leftPelvisIMUSensor = "v1Pelvis_LeftIMU";
   private static final String rightPelvisIMUSensor = "v1Pelvis_RightIMU";
   
   private static final HashMap<String, Integer> imuUSBSerialIds = new HashMap<>();
   static
   {
      imuUSBSerialIds.put(leftPelvisIMUSensor, 622730571);
      imuUSBSerialIds.put(rightPelvisIMUSensor, 622730566);
      imuUSBSerialIds.put(leftTrunkIMUSensor, 622730569);
      imuUSBSerialIds.put(rightTrunkIMUSensor, 622709817);
   }
   
   // Use this until sim can handle multiple IMUs
//    public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
//   public static final String[] imuSensorsToUse = {leftPelvisIMUSensor};
   public static final String[] imuSensorsToUse = {leftPelvisIMUSensor};
   
   public ValkyrieSensorInformation()
   {
      cameraParamaters[0] = new DRCRobotCameraParameters(forheadCameraName,forheadCameraTopic,headLinkName,forheadCameraId);
      cameraParamaters[1] = new DRCRobotCameraParameters(leftStereoCameraName,leftCameraTopic,headLinkName,leftHazardCameraId);
      cameraParamaters[2] = new DRCRobotCameraParameters(rightStereoCameraName,rightCameraTopic,headLinkName,rightHazardCameraId);
      pointCloudParamaters[IBEO_ID] = new DRCRobotPointCloudParameters(ibeoSensorName,ibeoTopic,headLinkName,IBEO_ID);
      }
   
   public static String getUrdfFeetForceSensorName(RobotSide side)
   {
      return urdfFeetForceSensorNames.get(side);
   }
   
   public HashMap<String, Integer> getImuUSBSerialIds()
   {
      return imuUSBSerialIds;
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUse;
   }
  
   @Override
   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return wristForceSensorNames;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return leftPelvisIMUSensor;
   }
   
   public RigidBodyTransform getTransformFromAnkleURDFFrameToZUpFrame(RobotSide robotSide)
   {
      return transformFromSixAxisMeasurementToAnkleZUpFrames.get(robotSide);
   }

   @Override
   public DRCRobotCameraParameters[] getCameraParameters()
   {
      return cameraParamaters;
   }

   @Override
   public DRCRobotCameraParameters getCameraParameters(int sensorId)
   {
      return cameraParamaters[sensorId];
   }

   @Override
   public DRCRobotLidarParameters[] getLidarParameters()
   {
      return new DRCRobotLidarParameters[0];
   }
   
   @Override
   public DRCRobotLidarParameters getLidarParameters(int sensorId)
   {
      return null;
   }

   @Override
   public DRCRobotPointCloudParameters[] getPointCloudParameters()
   {
      return pointCloudParamaters;
   }

   @Override
   public DRCRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return pointCloudParamaters[sensorId];
   }
   
   private void sensorFramesToTrack(DRCRobotSensorParameters[] sensorParams, ArrayList<String> holder)
   {
      for(int i = 0; i < sensorParams.length; i++)
      {
         if(sensorParams[i].getPoseFrameForSdf() != null)
         {
            holder.add(sensorParams[i].getPoseFrameForSdf());
         }
      }
   }
   
   @Override
   public String[] getSensorFramesToTrack()
   {
      ArrayList<String> sensorFramesToTrack = new ArrayList<String>();
      sensorFramesToTrack(cameraParamaters,sensorFramesToTrack);
      sensorFramesToTrack(pointCloudParamaters,sensorFramesToTrack);
      String[] sensorFramesToTrackAsPrimitive = new String[sensorFramesToTrack.size()];
      sensorFramesToTrack.toArray(sensorFramesToTrackAsPrimitive);
      return sensorFramesToTrackAsPrimitive;
   }

   @Override
   public boolean setupROSLocationService()
   {
      return false;
   }

   @Override
   public boolean setupROSParameterSetters()
   {
      return false;
   }

   @Override
   public boolean isMultisenseHead()
   {
      return false;
   }

   @Override
   public ReferenceFrame getHeadIMUFrameWhenLevel()
   {
      return null;
   }

   public String getRightTrunkIMUSensor()
   {
      return rightTrunkIMUSensor;
   }

   public String getLeftTrunkIMUSensor()
   {
      return leftTrunkIMUSensor;
   }

   public String getLeftPelvisIMUSensor()
   {
      return leftPelvisIMUSensor;
   }

   public String getRightPelvisIMUSensor()
   {
      return rightPelvisIMUSensor;
   }
}
