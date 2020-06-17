package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;

public interface OcTreeConsumer
{
   void reportOcTree(NormalOcTree ocTree, Tuple3DReadOnly sensorPosition);
}
