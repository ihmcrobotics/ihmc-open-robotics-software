package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;

public interface OcTreeConsumer
{
   void reportOcTree(NormalOcTree ocTree, Pose3DReadOnly sensorPose);
}
