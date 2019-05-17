package us.ihmc.sensorProcessing.parameters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface AvatarHumanoidRobotSensorInformation
{
   public String[] getIMUSensorsToUseInStateEstimator();

   public String[] getForceSensorNames();

   public SideDependentList<String> getFeetForceSensorNames();

   public SideDependentList<String> getFeetContactSensorNames();

   public SideDependentList<String> getWristForceSensorNames();

   public String getPrimaryBodyImu();

   public AvatarRobotCameraParameters[] getCameraParameters();

   public AvatarRobotLidarParameters[] getLidarParameters();

   public AvatarRobotPointCloudParameters[] getPointCloudParameters();

   public AvatarRobotCameraParameters getCameraParameters(int cameraId);

   public AvatarRobotLidarParameters getLidarParameters(int lidarId);

   public AvatarRobotPointCloudParameters getPointCloudParameters(int pointCloudSensorId);

   @Deprecated // Unused, remove.
   public ReferenceFrame getHeadIMUFrameWhenLevel();

   public String[] getSensorFramesToTrack();

   public boolean setupROSLocationService();

   public boolean setupROSParameterSetters();

   public boolean isMultisenseHead();

   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos();
}
