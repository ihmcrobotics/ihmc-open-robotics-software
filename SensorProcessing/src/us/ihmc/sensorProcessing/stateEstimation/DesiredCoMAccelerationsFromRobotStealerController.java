package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DesiredCoMAccelerationsFromRobotStealerController implements RobotController, DesiredCoMAndAngularAccelerationOutputPortsHolder
{
   private static final boolean CORRUPT_DESIRED_ACCELERATIONS = true;

   private double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint perfectCoM = new YoFramePoint("perfectCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectCoMd = new YoFrameVector("perfectCoMd", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector perfectCoMdd = new YoFrameVector("perfectCoMdd", ReferenceFrame.getWorldFrame(), registry);

   private final InverseDynamicsJointsFromSCSRobotGenerator generator;
   private final TwistCalculator perfectTwistCalculator;
   private final SpatialAccelerationCalculator perfectSpatialAccelerationCalculator;

   private final CenterOfMassCalculator perfectCenterOfMassCalculator;
   private final CenterOfMassJacobian perfectCenterOfMassJacobian;
   private final CenterOfMassAccelerationCalculator perfectCenterOfMassAccelerationCalculator;

   private final double controlDT;

   private final ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort;
   private final ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort;

   public DesiredCoMAccelerationsFromRobotStealerController(SensorNoiseParameters sensorNoiseParameters, 
         InverseDynamicsJointsFromSCSRobotGenerator generator, Joint estimationJoint, double controlDT)
   {
      this.comAccelerationProcessNoiseStandardDeviation = sensorNoiseParameters.getComAccelerationProcessNoiseStandardDeviation();
      this.angularAccelerationProcessNoiseStandardDeviation = sensorNoiseParameters.getAngularAccelerationProcessNoiseStandardDeviation();
      
      this.controlDT = controlDT;

      this.generator = generator;
      RigidBody elevator = generator.getElevator();

      perfectTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      perfectSpatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, perfectTwistCalculator, 0.0, false);

      perfectCenterOfMassCalculator = new CenterOfMassCalculator(elevator, ReferenceFrame.getWorldFrame());
      perfectCenterOfMassJacobian = new CenterOfMassJacobian(elevator);
      perfectCenterOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, perfectSpatialAccelerationCalculator);

      CenterOfMassAccelerationFromFullRobotModelStealer centerOfMassAccelerationFromFullRobotModelStealer =
         new CenterOfMassAccelerationFromFullRobotModelStealer(elevator, perfectSpatialAccelerationCalculator);
      desiredCenterOfMassAccelerationOutputPort = centerOfMassAccelerationFromFullRobotModelStealer.getOutputPort();

      AngularAccelerationFromRobotStealer angularAccelerationFromRobotStealer = new AngularAccelerationFromRobotStealer(estimationJoint);
      desiredAngularAccelerationOutputPort = angularAccelerationFromRobotStealer.getOutputPort();
   }


   public ControlFlowOutputPort<FrameVector> getDesiredCenterOfMassAccelerationOutputPort()
   {
      return desiredCenterOfMassAccelerationOutputPort;
   }

   public ControlFlowOutputPort<FrameVector> getDesiredAngularAccelerationOutputPort()
   {
      return desiredAngularAccelerationOutputPort;
   }

   public void doControl()
   {
      boolean updateRootJoints = true;
      generator.updateInverseDynamicsRobotModelFromRobot(updateRootJoints, false);

      perfectTwistCalculator.compute();
      perfectSpatialAccelerationCalculator.compute();

      updateGroundTruth();
   }

   private void updateGroundTruth()
   {
      FramePoint com = new FramePoint();
      perfectCenterOfMassCalculator.compute();
      perfectCenterOfMassCalculator.packCenterOfMass(com);
      perfectCoM.set(com);

      FrameVector comd = new FrameVector();
      perfectCenterOfMassJacobian.compute();
      perfectCenterOfMassJacobian.packCenterOfMassVelocity(comd);
      comd.changeFrame(ReferenceFrame.getWorldFrame());
      perfectCoMd.set(comd);

      FrameVector comdd = new FrameVector();
      perfectCenterOfMassAccelerationCalculator.packCoMAcceleration(comdd);
      comdd.changeFrame(ReferenceFrame.getWorldFrame());
      perfectCoMdd.set(comdd);
   }


   private class CenterOfMassAccelerationFromFullRobotModelStealer extends AbstractControlFlowElement
   {
      private final CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator;
      private final FrameVector comAccelerationFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
      private final Vector3d comAcceleration = new Vector3d();

      private final ControlFlowOutputPort<FrameVector> outputPort = createOutputPort();
      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public CenterOfMassAccelerationFromFullRobotModelStealer(RigidBody rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator)
      {
         this.centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(rootBody, spatialAccelerationCalculator);

         double discreteStdDev = convertStandardDeviationToDiscreteTime(comAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public ControlFlowOutputPort<FrameVector> getOutputPort()
      {
         return outputPort;
      }

      public void startComputation()
      {
         centerOfMassAccelerationCalculator.packCoMAcceleration(comAccelerationFrameVector);
         comAccelerationFrameVector.changeFrame(ReferenceFrame.getWorldFrame());
         comAccelerationFrameVector.getVector(comAcceleration);

         if (CORRUPT_DESIRED_ACCELERATIONS)
            signalCorruptor.corrupt(comAcceleration);

         comAccelerationFrameVector.set(comAcceleration);
         outputPort.setData(comAccelerationFrameVector);
      }

      public void waitUntilComputationIsDone()
      {
      }
   }



   private class AngularAccelerationFromRobotStealer extends AbstractControlFlowElement
   {
      private final Joint estimationJoint;
      private final ControlFlowOutputPort<FrameVector> outputPort = createOutputPort();

      // Note: Null reference frame!
      // TODO: This should probably just have a Vector3d port rather than a FrameVector port. Since we don't
      // know the frame that the user will want it in. It should be implied to be a body frame.
      // Or that frame should be passed in somehow.
      private final FrameVector desiredAngularAccelerationFrameVector = new FrameVector((ReferenceFrame) null);
      private final Vector3d desiredAngularAcceleration = new Vector3d();

      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public AngularAccelerationFromRobotStealer(Joint estimationJoint)
      {
         this.estimationJoint = estimationJoint;
         double discreteStdDev = convertStandardDeviationToDiscreteTime(angularAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public void startComputation()
      {
         estimationJoint.getAngularAccelerationsInBodyFrame(desiredAngularAcceleration);

         if (CORRUPT_DESIRED_ACCELERATIONS)
            signalCorruptor.corrupt(desiredAngularAcceleration);

         // TODO: Null frame?
         desiredAngularAccelerationFrameVector.set(null, desiredAngularAcceleration);
         outputPort.setData(desiredAngularAccelerationFrameVector);
      }

      public ControlFlowOutputPort<FrameVector> getOutputPort()
      {
         return outputPort;
      }

      public void waitUntilComputationIsDone()
      {
      }

   }


   private double convertStandardDeviationToDiscreteTime(double continuousStdDev)
   {
      double continuousVariance = MathTools.square(continuousStdDev);
      double discreteVariance = continuousVariance * controlDT;
      double discreteStdDev = Math.sqrt(discreteVariance);

      return discreteStdDev;
   }

   public void initialize()
   {
      // Do nothing.
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

}
