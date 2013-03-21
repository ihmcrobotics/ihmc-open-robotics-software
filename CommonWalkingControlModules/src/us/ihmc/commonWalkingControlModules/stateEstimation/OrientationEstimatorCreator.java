package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class OrientationEstimatorCreator
{
   private final RigidBody orientationEstimationLink;
   private final TwistCalculator twistCalculator;

   private final List<RigidBody> angularVelocityMeasurementLinks = new ArrayList<RigidBody>();
   private final List<ReferenceFrame> angularVelocityMeasurementFrames = new ArrayList<ReferenceFrame>();

   public OrientationEstimatorCreator(RigidBody orientationEstimationLink, TwistCalculator twistCalculator)
   {
      this.orientationEstimationLink = orientationEstimationLink;
      this.twistCalculator = twistCalculator;
   }

   public void addAngularVelocitySensorConfiguration(RigidBody measurementLink, ReferenceFrame measurementFrame)
   {
      angularVelocityMeasurementLinks.add(measurementLink);
      angularVelocityMeasurementFrames.add(measurementFrame);
   }

   public void addOrientationSensorConfiguration()
   {
      // TODO: pattern match addAngularVelocitySensorConfiguration
   }

   public void createOrientationEstimator(double controlDT, ReferenceFrame estimationFrame, YoVariableRegistry registry)
   {
      ComposableStateEstimator ret = new ComposableStateEstimator("orientationEstimator", controlDT);

      // states
      ControlFlowOutputPort<FrameOrientation> orientationPort = ret.createStatePort(3);
      ControlFlowOutputPort<FrameVector> angularVelocityPort = ret.createStatePort(3);

      // process inputs
      ControlFlowInputPort<FrameVector> angularAccelerationPort = ret.createProcessInputPort(3);

      // measurements
      ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = ret.createMeasurementInputPort(3);

      // process model
      ProcessModelElement orientationProcessModelElement = new OrientationProcessModelElement(angularVelocityPort, orientationPort, "orientation", registry);
      ret.addProcessModelElement(orientationPort, orientationProcessModelElement);

      ProcessModelElement angularVelocityProcessModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityPort,
                                                                  angularAccelerationPort, "angularVelocity", registry);
      ret.addProcessModelElement(angularVelocityPort, angularVelocityProcessModelElement);


      // measurement model
      for (int i = 0; i < angularVelocityMeasurementLinks.size(); i++)
      {
         ControlFlowOutputPort<FrameVector> biasPort = ret.createStatePort(3);
         ProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, "bias" + i, registry);
         ret.addProcessModelElement(biasPort, biasProcessModelElement);

         RigidBody measurementLink = angularVelocityMeasurementLinks.get(i);
         ReferenceFrame measurementFrame = angularVelocityMeasurementFrames.get(i);

         MeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityPort, biasPort,
                                                                      angularVelocityMeasurementPort, orientationEstimationLink, measurementLink,
                                                                      measurementFrame, twistCalculator, "angularVelocitymeasurement" + i, registry);
         ret.addMeasurementModelElement(angularVelocityMeasurementPort, angularVelocityMeasurementModel);
      }
      
      // TODO: add orientation sensors

      ret.initialize();
   }
}
