package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Interface to provide perfect sensor information of the root joint   
 * @author jesper
 *
 */
public interface RootJointPerfectSensorOutputMapReadOnly
{
   public void packRootJointTransform(RigidBodyTransform transformToPack);
   
   public void packRootJointLinearVelocityInBody(Vector3D linearVelocityToPack);
   public void packRootJointAngularVelocityInBody(Vector3D angularVelocityToPack);
}
