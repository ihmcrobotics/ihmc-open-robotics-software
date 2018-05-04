package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulatedSensor;

public class LidarMount implements SimulatedSensor
{
   
   private final LidarSensorDescription description;
   protected RigidBodyTransform transformToHere = new RigidBodyTransform();
   private final RigidBodyTransform transformFromJoint;
   private GPULidar lidar;
   private final String lidarName;

   private Joint parentJoint;
   
   public LidarMount(LidarSensorDescription description)
   {
      this.description = description;
      this.transformFromJoint = new RigidBodyTransform(description.getTransformToJoint());
      this.lidarName = description.getName();
   }

   @Override
   public void updateTransform(RigidBodyTransform transformToHere, double time)
   {
      this.transformToHere.set(transformToHere);
      this.transformToHere.multiply(transformFromJoint);

      if(lidar != null)
      {
         lidar.setTransformFromWorld(this.transformToHere, time);
      }
   }
   
   @Override
   public String getName()
   {
      return lidarName;
   }
   
   public void setLidar(GPULidar lidar)
   {
      this.lidar = lidar;
   }

   public LidarSensorDescription getDescription()
   {
      return description;
   }
   
   @Override
   public void setWorld(Graphics3DAdapter graphics3dAdapter)
   {
      
   }
   
   public void setParentJoint(Joint parent)
   {
      this.parentJoint = parent;
   }

   public Joint getParentJoint()
   {
      return parentJoint;
   }

   @Override
   public RigidBodyTransform getTransformToHere()
   {
      return transformToHere;
   }

   public void updateTransformFromJoint(RigidBodyTransform transformFromJoint)
   {
      this.transformFromJoint.multiply(transformFromJoint);
   }
}
