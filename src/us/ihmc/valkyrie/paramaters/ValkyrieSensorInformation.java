package us.ihmc.valkyrie.paramaters;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
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
   private static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("LeftAnkle", "RightAnkle");
   private static final SideDependentList<String> urdfFeetForceSensorNames = new SideDependentList<>("/v1/LeftLeg6Axis_Offset", "/v1/RightLeg6Axis_Offset");

   public static final boolean USE_JSC_FOOT_MASS_TARING = false;

   private static final SideDependentList<SpatialForceVector> footForceSensorTareOffsets;
   static
   {
//      SpatialForceVector leftFootForceSensorTareOffset_20140406 = new SpatialForceVector(null, new double[] { 7.1, -23.7, 3.8, 186.2, 319.7, 1067.7 });
//      SpatialForceVector rightFootForceSensorTareOffset_20140406 = new SpatialForceVector(null, new double[] { -1.08, -2.79, -9.63, 38.05, 7.35, -109.3 });

//      SpatialForceVector leftFootForceSensorTareOffset_20140512 = new SpatialForceVector(null, new double[] { 7.02, -23.79, 3.09, 189.6, 322.5, 1081.0 });
//      SpatialForceVector rightFootForceSensorTareOffset_20140512 = new SpatialForceVector(null, new double[] {-1.12, -2.46, -8.94, 27.54, 3.70, -101.7});

//      SpatialForceVector leftFootForceSensorTareOffset_20140903 = new SpatialForceVector(null, new double[] { 7.1, -24.2, 1.96, 187.9, 306.0, 1131.6 });
//      SpatialForceVector rightFootForceSensorTareOffset_20140903 = new SpatialForceVector(null, new double[] { -1.0, -2.7, -10.1, 61.1, 16.3, -143.0 });

//      SpatialForceVector leftFootForceSensorTareOffset_20140909 = new SpatialForceVector(null, new double[] {6.6, -23.4, 2.2, 193.9, 298.5, 1142});
//      SpatialForceVector rightFootForceSensorTareOffset_20140909 = new SpatialForceVector(null, new double[] {-1.5, -2.8, -10.8, 66.0, -2.0, -156.0});

      SpatialForceVector leftFootForceSensorTareOffset_20140916 = new SpatialForceVector(null, new double[] {6.98, -24.8, 2.01, 175.0, 313.0, 1102.0});
      SpatialForceVector rightFootForceSensorTareOffset_20140916 = new SpatialForceVector(null, new double[] {-1.42, -2.89, -11.2, 83.0, -15.0, -138.2});

      
      footForceSensorTareOffsets = new SideDependentList<SpatialForceVector>(leftFootForceSensorTareOffset_20140916, rightFootForceSensorTareOffset_20140916);
   }

   public static final SideDependentList<RigidBodyTransform> transformFromMeasurementToAnkleZUpFrames = new SideDependentList<>();
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

      transformFromMeasurementToAnkleZUpFrames.put(RobotSide.LEFT, leftTransform);
      transformFromMeasurementToAnkleZUpFrames.put(RobotSide.RIGHT, new RigidBodyTransform(leftTransform));
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
   private static final String fakePelvisIMUSensor = "v1Pelvis_SimulatedIMU";
   
   private static final HashMap<String, Integer> imuUSBSerialIds = new HashMap<>();
   static
   {
      imuUSBSerialIds.put("v1Pelvis_LeftIMU", 622730566);
      imuUSBSerialIds.put("v1Pelvis_RightIMU", 622730571);
   }
   
   // Use this until sim can handle multiple IMUs
    public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
//   public static final String[] imuSensorsToUse = {leftPelvisIMUSensor};
//   public static final String[] imuSensorsToUse = {rightPelvisIMUSensor};
   
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
   public String[] getIMUSensorsToUse()
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

   public static SpatialForceVector getFootForceSensorTareOffset(RobotSide robotSide)
   {
      return footForceSensorTareOffsets.get(robotSide);
   }
   
   public RigidBodyTransform getTransformFromAnkleURDFFrameToZUpFrame(RobotSide robotSide)
   {
      return transformFromMeasurementToAnkleZUpFrames.get(robotSide);
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
}
