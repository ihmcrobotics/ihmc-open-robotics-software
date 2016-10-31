package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.robotics.math.corruptors.NoisyDoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class PerfectSimulatedIMURawSensorReader extends SimulatedIMURawSensorReader
{
   public PerfectSimulatedIMURawSensorReader(RawIMUSensorsInterface rawSensors, int imuIndex, RigidBody rigidBody, ReferenceFrame imuFrame, RigidBody rootBody, SpatialAccelerationVector rootAcceleration)
   {
      super(rawSensors, imuIndex, rigidBody, imuFrame, rootBody, rootAcceleration);
   }

   @Override
   protected void initializeNoise()
   {
      rotationMatrix.setIsNoisy(false);
      setIsNoisyToFalse(accelList);
      setIsNoisyToFalse(gyroList);
      setIsNoisyToFalse(compassList);
   }
   
   @Override
   protected void simulateIMU()
   {
      rotationMatrix.update(perfM00.getDoubleValue(), perfM01.getDoubleValue(), perfM02.getDoubleValue(), perfM10.getDoubleValue(), perfM11.getDoubleValue(), perfM12.getDoubleValue(), perfM20.getDoubleValue(), perfM21.getDoubleValue(), perfM22.getDoubleValue());
      
      accelX.update();
      accelY.update();
      accelZ.update();
      
      gyroX.update();
      gyroY.update();
      gyroZ.update();
      
      compassX.update();
      compassY.update();
      compassZ.update();
   }

   private void setIsNoisyToFalse(NoisyDoubleYoVariable[] list)
   {
      for (NoisyDoubleYoVariable i : list)
      {
         i.setIsNoisy(false);
      }
   }
}