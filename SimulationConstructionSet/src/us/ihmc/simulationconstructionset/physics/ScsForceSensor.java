package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;


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
