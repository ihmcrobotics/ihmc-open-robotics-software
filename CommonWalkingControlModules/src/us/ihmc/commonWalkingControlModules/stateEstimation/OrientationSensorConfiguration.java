package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.ControlFlowPort;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class OrientationSensorConfiguration<PortType extends ControlFlowPort<Matrix3d>> extends SensorConfiguration<Matrix3d, PortType>
{
   private static final int ROTATION_VECTOR_LENGTH = 3;

   private final Map<PortType, ReferenceFrame> measurementFrames = new HashMap<PortType, ReferenceFrame>();

   public OrientationSensorConfiguration()
   {
      super(ROTATION_VECTOR_LENGTH);
   }
   
   public void addSensor(PortType measurementPort, ReferenceFrame measurementFrame, String sensorName, DenseMatrix64F covariance)
   {
      measurementPorts.add(measurementPort);
      measurementFrames.put(measurementPort, measurementFrame);
      sensorNames.put(measurementPort, sensorName);
      covariances.put(measurementPort, covariance);
   }

   public ReferenceFrame getReferenceFrame(PortType port)
   {
      return measurementFrames.get(port);
   }
   
   public ArrayList<NewOrientationSensorConfiguration> getNewOrientationSensorConfiguration()
   {
      ArrayList<NewOrientationSensorConfiguration> ret = new ArrayList<NewOrientationSensorConfiguration>();
      
      for (PortType measurementPort : measurementPorts)
      {
         ControlFlowOutputPort<Matrix3d> outputPort = (ControlFlowOutputPort<Matrix3d>) measurementPort;
         String name = sensorNames.get(measurementPort);
         ReferenceFrame measurementFrame = measurementFrames.get(measurementPort);
         DenseMatrix64F orientationNoiseCovariance = covariances.get(measurementPort);
         NewOrientationSensorConfiguration newOrientationSensorConfiguration = new NewOrientationSensorConfiguration(outputPort, name, measurementFrame, orientationNoiseCovariance);

         ret.add(newOrientationSensorConfiguration);
      }
      
      return ret;
   }
}
