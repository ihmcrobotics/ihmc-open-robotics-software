package us.ihmc.simulationConstructionSetTools.simulatedSensors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.robotics.math.corruptors.NoisyYoDouble;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class PerfectSimulatedIMURawSensorReader extends SimulatedIMURawSensorReader
{
   public PerfectSimulatedIMURawSensorReader(RawIMUSensorsInterface rawSensors, int imuIndex, RigidBodyBasics rigidBody, ReferenceFrame imuFrame, RigidBodyBasics rootBody, SpatialAccelerationReadOnly rootAcceleration)
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

   private void setIsNoisyToFalse(NoisyYoDouble[] list)
   {
      for (NoisyYoDouble i : list)
      {
         i.setIsNoisy(false);
      }
   }
}