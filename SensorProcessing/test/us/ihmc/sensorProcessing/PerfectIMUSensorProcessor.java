package us.ihmc.sensorProcessing;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.processedSensors.ProcessedIMUSensorsWriteOnlyInterface;
import us.ihmc.simulationconstructionset.rawSensors.RawIMUSensorsInterface;
import us.ihmc.simulationconstructionset.robotController.SensorProcessor;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PerfectIMUSensorProcessor implements SensorProcessor
{
   private final String name;
   private final YoVariableRegistry registry;
   
   private final RawIMUSensorsInterface rawIMUSensors;
   private final ProcessedIMUSensorsWriteOnlyInterface processedIMUSensors;
   private final int imuIndex = 0;

   private final Matrix3d rotationMatrix = new Matrix3d();
   private final Vector3d angRate = new Vector3d();
   private final Vector3d acceleration = new Vector3d();
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
      rawIMUSensors.packOrientation(rotationMatrix, imuIndex);
      processedIMUSensors.setRotation(rotationMatrix, imuIndex);
   }

   private void processAngularVelocity()
   {
      rawIMUSensors.packAngularVelocity(angRate, imuIndex);
      processedIMUSensors.setAngularVelocityInBody(angRate, imuIndex);
   }

   private void processAcceleration()
   {
      rawIMUSensors.packAcceleration(acceleration, imuIndex);

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