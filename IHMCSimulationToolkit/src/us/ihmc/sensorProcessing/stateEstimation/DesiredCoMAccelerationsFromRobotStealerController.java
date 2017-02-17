package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.simulationconstructionset.Joint;

public class DesiredCoMAccelerationsFromRobotStealerController implements RobotController
{
   private static final boolean CORRUPT_DESIRED_ACCELERATIONS = true;

   private double comAccelerationProcessNoiseStandardDeviation;
   private double angularAccelerationProcessNoiseStandardDeviation;

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

   private StateEstimationDataFromController desiredCoMAndAngularAccelerationDataSource;
   private final CenterOfMassAccelerationFromFullRobotModelStealer centerOfMassAccelerationFromFullRobotModelStealer;
   private final AngularAccelerationFromRobotStealer angularAccelerationFromRobotStealer;
      
   private final ReferenceFrame estimationFrame;
   
   public DesiredCoMAccelerationsFromRobotStealerController(ReferenceFrame estimationFrame,
         double comAccelerationProcessNoiseStandardDeviation,
         double angularAccelerationProcessNoiseStandardDeviation, 
         InverseDynamicsJointsFromSCSRobotGenerator generator, Joint estimationJoint, double controlDT)
   {
      this.estimationFrame = estimationFrame;
      this.comAccelerationProcessNoiseStandardDeviation = comAccelerationProcessNoiseStandardDeviation;
      this.angularAccelerationProcessNoiseStandardDeviation = angularAccelerationProcessNoiseStandardDeviation;
      
      this.controlDT = controlDT;

      this.generator = generator;
      RigidBody elevator = generator.getElevator();

      perfectTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      perfectSpatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, perfectTwistCalculator, 0.0, false);

      perfectCenterOfMassCalculator = new CenterOfMassCalculator(elevator, ReferenceFrame.getWorldFrame());
      perfectCenterOfMassJacobian = new CenterOfMassJacobian(elevator);
      perfectCenterOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, perfectSpatialAccelerationCalculator);

      centerOfMassAccelerationFromFullRobotModelStealer =
         new CenterOfMassAccelerationFromFullRobotModelStealer(elevator, perfectSpatialAccelerationCalculator);

      angularAccelerationFromRobotStealer = new AngularAccelerationFromRobotStealer(estimationJoint);
   }

   public void doControl()
   {
      boolean updateRootJoints = true;
      generator.updateInverseDynamicsRobotModelFromRobot(updateRootJoints, false);

      perfectTwistCalculator.compute();
      perfectSpatialAccelerationCalculator.compute();

      updateGroundTruth();
      
      centerOfMassAccelerationFromFullRobotModelStealer.run();
      angularAccelerationFromRobotStealer.run();
   }

   private void updateGroundTruth()
   {
      FramePoint com = new FramePoint();
      perfectCenterOfMassCalculator.compute();
      perfectCenterOfMassCalculator.getCenterOfMass(com);
      perfectCoM.set(com);

      FrameVector comd = new FrameVector();
      perfectCenterOfMassJacobian.compute();
      perfectCenterOfMassJacobian.getCenterOfMassVelocity(comd);
      comd.changeFrame(ReferenceFrame.getWorldFrame());
      perfectCoMd.set(comd);

      FrameVector comdd = new FrameVector();
      perfectCenterOfMassAccelerationCalculator.getCoMAcceleration(comdd);
      comdd.changeFrame(ReferenceFrame.getWorldFrame());
      perfectCoMdd.set(comdd);
   }


   private class CenterOfMassAccelerationFromFullRobotModelStealer implements Runnable
   {
      private final CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator;
      private final FrameVector comAccelerationFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
      private final Vector3D comAcceleration = new Vector3D();

      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public CenterOfMassAccelerationFromFullRobotModelStealer(RigidBody rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator)
      {
         this.centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(rootBody, spatialAccelerationCalculator);

         double discreteStdDev = convertStandardDeviationToDiscreteTime(comAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public void run()
      {
         centerOfMassAccelerationCalculator.getCoMAcceleration(comAccelerationFrameVector);
         comAccelerationFrameVector.changeFrame(ReferenceFrame.getWorldFrame());
         comAccelerationFrameVector.get(comAcceleration);

         if (CORRUPT_DESIRED_ACCELERATIONS)
            signalCorruptor.corrupt(comAcceleration);

         comAccelerationFrameVector.set(comAcceleration);
         
         desiredCoMAndAngularAccelerationDataSource.setDesiredCenterOfMassAcceleration(comAccelerationFrameVector);
      }
   }



   private class AngularAccelerationFromRobotStealer implements Runnable
   {
      private final Joint estimationJoint;

      // Note: Null reference frame!
      // TODO: This should probably just have a Vector3d port rather than a FrameVector port. Since we don't
      // know the frame that the user will want it in. It should be implied to be a body frame.
      // Or that frame should be passed in somehow.
      private final FrameVector desiredAngularAccelerationFrameVector = new FrameVector((ReferenceFrame) null);
      private final Vector3D desiredAngularAcceleration = new Vector3D();

      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public AngularAccelerationFromRobotStealer(Joint estimationJoint)
      {
         this.estimationJoint = estimationJoint;
         double discreteStdDev = convertStandardDeviationToDiscreteTime(angularAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public void run()
      {
         estimationJoint.physics.getAngularAccelerationsInBodyFrame(desiredAngularAcceleration);

         if (CORRUPT_DESIRED_ACCELERATIONS)
            signalCorruptor.corrupt(desiredAngularAcceleration);

         desiredAngularAccelerationFrameVector.setIncludingFrame(estimationFrame, desiredAngularAcceleration);
         
         desiredCoMAndAngularAccelerationDataSource.setDesiredAngularAcceleration(desiredAngularAccelerationFrameVector);
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
   
   public void attachStateEstimationDataFromControllerSink(StateEstimationDataFromController desiredCoMAndAngularAccelerationDataSource)
   {
      this.desiredCoMAndAngularAccelerationDataSource = desiredCoMAndAngularAccelerationDataSource;
   }


}
