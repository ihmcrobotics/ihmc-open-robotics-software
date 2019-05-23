package us.ihmc.sensorProcessing.parameters;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.ArrayList;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface AvatarRobotRosIntegratedSensorInformation
{
   public boolean setupROSLocationService();

   public boolean setupROSParameterSetters();

   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos();
}
