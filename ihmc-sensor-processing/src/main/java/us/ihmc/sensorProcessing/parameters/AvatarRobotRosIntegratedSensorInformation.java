package us.ihmc.sensorProcessing.parameters;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface AvatarRobotRosIntegratedSensorInformation
{
   boolean setupROSLocationService();

   boolean setupROSParameterSetters();

   default List<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return Collections.emptyList();
   }
}
