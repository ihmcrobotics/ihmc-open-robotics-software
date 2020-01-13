package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam;

import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface IhmcSLAMInterface
{
   abstract void addFirstFrame(StereoVisionPointCloudMessage pointCloudMessage);

   abstract boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage);

   abstract void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage);

   abstract void updatePlanarRegionsMap();

   abstract List<Point3DReadOnly[]> getOriginalPointCloudMap();

   abstract List<RigidBodyTransformReadOnly> getOriginalSensorPoses();

   abstract List<Point3DReadOnly[]> getPointCloudMap();

   abstract List<RigidBodyTransformReadOnly> getSensorPoses();

   abstract PlanarRegionsList getPlanarRegionsMap();
}
