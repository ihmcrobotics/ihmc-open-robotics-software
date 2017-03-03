package us.ihmc.sensorProcessing;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.SensorProcessor;
import us.ihmc.robotics.sensors.ProcessedIMUSensorsWriteOnlyInterface;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class PerfectIMUSensorProcessor implements SensorProcessor
{
   private final String name;
   private final YoVariableRegistry registry;
   
   private final RawIMUSensorsInterface rawIMUSensors;
   private final ProcessedIMUSensorsWriteOnlyInterface processedIMUSensors;
   private final int imuIndex = 0;

   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector3D angRate = new Vector3D();
   private final Vector3D acceleration = new Vector3D();
   private final FrameVector accelerationInWorld = new FrameVector(ReferenceFrame.getWorldFrame());

   public PerfectIMUSensorProcessor(RawIMUSensorsInterface rawIMUSensors, ProcessedIMUSensorsWriteOnlyInterface processedIMUSensors)
   {
      this.rawIMUSensors = rawIMUSensors;
      this.processedIMUSensors = processedIMUSensors;
      
      name = getClass().getSimpleName() + imuIndex;
      registry = new YoVariableRegistry(name);
   }

   public void initialize()
   {
      update();
   }

   public void update()
   {
      processOrientation();
      processAngularVelocity();
      processAcceleration();
   }

   private void processOrientation()
   {
      rawIMUSensors.getOrientation(rotationMatrix, imuIndex);
      processedIMUSensors.setRotation(rotationMatrix, imuIndex);
   }

   private void processAngularVelocity()
   {
      rawIMUSensors.getAngularVelocity(angRate, imuIndex);
      processedIMUSensors.setAngularVelocityInBody(angRate, imuIndex);
   }

   private void processAcceleration()
   {
      rawIMUSensors.getAcceleration(acceleration, imuIndex);

      rotationMatrix.transform(acceleration); // Compute acceleration in world frame
      acceleration.setZ(acceleration.getZ() + 0.0); // Compensate gravity
      accelerationInWorld.set(acceleration);

      processedIMUSensors.setAcceleration(accelerationInWorld, imuIndex);
   }
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return name;
   }
}