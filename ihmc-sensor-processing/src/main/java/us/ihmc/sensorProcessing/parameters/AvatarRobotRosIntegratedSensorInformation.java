package us.ihmc.sensorProcessing.parameters;

import java.util.Collections;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface AvatarRobotRosIntegratedSensorInformation
{
   default boolean setupROSLocationService()
   {
      return false;
   }

   default boolean setupROSParameterSetters()
   {
      return false;
   }

   default List<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return Collections.emptyList();
   }
}
