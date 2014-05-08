package us.ihmc.valkyrie.paramaters;

import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class ValkyrieSensorInformation implements DRCRobotSensorInformation
{

   public static final String[] forceSensorNames = { "LeftAnkle", "RightAnkle", "LeftForearmSupinator", "RightForearmSupinator" };
   private static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("LeftAnkle", "RightAnkle");
   private static final SideDependentList<String> urdfFeetForceSensorNames = new SideDependentList<>("/v1/LeftLeg6Axis_Offset", "/v1/RightLeg6Axis_Offset");

   public static final boolean USE_JSC_FOOT_MASS_TARING = false;
   public static final boolean USE_HOME_MADE_FOOT_SENSOR_TARRING = true;

   private static final SideDependentList<SpatialForceVector> footForceSensorTareOffsets;
   static
   {
      SpatialForceVector leftFootForceSensorTareOffset_20140406 = new SpatialForceVector(null, new double[] { 7.1, -23.7, 3.8, 186.2, 319.7, 1067.7 });
      SpatialForceVector rightFootForceSensorTareOffset_20140406 = new SpatialForceVector(null, new double[] { -1.08, -2.79, -9.63, 38.05, 7.35, -109.3 });
      
      footForceSensorTareOffsets = new SideDependentList<SpatialForceVector>(leftFootForceSensorTareOffset_20140406, rightFootForceSensorTareOffset_20140406);
   }

   public static final SideDependentList<Transform3D> transformFromAnkleURDFFrameToZUpFrames = new SideDependentList<>();
   static
   {
      Transform3D leftTransform = new Transform3D();
      leftTransform.setEuler(new Vector3d(-Math.PI, Math.PI / 2.0, 0.0));
      transformFromAnkleURDFFrameToZUpFrames.put(RobotSide.LEFT, leftTransform);

      Transform3D rightTransform = new Transform3D();
      rightTransform.setEuler(new Vector3d(0.0, Math.PI / 2.0, 0.0));
      transformFromAnkleURDFFrameToZUpFrames.put(RobotSide.RIGHT, rightTransform);
   }

   
   private static final SideDependentList<String> wristForceSensorNames = new SideDependentList<String>("LeftForearmSupinator", "RightForearmSupinator");
   private static final String lidarSensorName = "/v1/Ibeo_sensor";
   private static final String lidarJointName = "";
   private static final String leftCameraName = "/v1/LeftHazardCamera___default__";
   private static final String rightCameraName ="/v1/RightHazardCamera___default__";
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
   // public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
   public static final String[] imuSensorsToUse = {leftPelvisIMUSensor};
   
   public String getUrdfFeetForceSensorName(RobotSide side)
   {
      return urdfFeetForceSensorNames.get(side);
   }
   
   public HashMap<String, Integer> getImuUSBSerialIds()
   {
      return imuUSBSerialIds;
   }
   
   @Override
   public String getLeftCameraName()
   {
      return leftCameraName;
   }

   @Override
   public String getRightCameraName()
   {
      return rightCameraName;
   }

   @Override
   public String getLidarSensorName()
   {
      return lidarSensorName;
   }

   @Override
   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
   }

   @Override
   public String getLidarJointName()
   {
      return lidarJointName;
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

   public SpatialForceVector getFootForceSensorTareOffset(RobotSide robotSide)
   {
      return footForceSensorTareOffsets.get(robotSide);
   }
   
   public Transform3D getTransformFromAnkleURDFFrameToZUpFrame(RobotSide robotSide)
   {
      return transformFromAnkleURDFFrameToZUpFrames.get(robotSide);
   }
}
