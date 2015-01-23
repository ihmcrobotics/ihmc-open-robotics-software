package us.ihmc.simulationconstructionset.physics;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


/**
 * Returns the measured force in sensor coordinates.
 *
 * @author Peter Abeles
 */
public interface ScsForceSensor
{

   public CollisionShape getShape();

   /**
    * Transform from sensor to shape coordinates.
    */
   public RigidBodyTransform getSensorToShape();


   public void getTransformToWorld( RigidBodyTransform sensorToWorld);

   public String getName();

   public DoubleYoVariable getYoForceX();
   public DoubleYoVariable getYoForceY();
   public DoubleYoVariable getYoForceZ();
}
