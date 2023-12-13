package us.ihmc.sensorProcessing.sensorData;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ForceSensorDistalMassCompensator
{
   private final double GRAVITY = 9.81;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame sensorFrame;

   private final FramePose3D sensorPose;
   private final YoFramePoint3D yoSensorPositionInWorld;

   private final CenterOfMassCalculator distalMassCalc;
   private final YoDouble distalMass;
   private final AlphaFilteredYoVariable lowPassSensorForceZ;
   private final YoFrameVector3D distalMassForceInWorld;
   private final YoFramePoint3D distalCoMInWorld;

   private final YoFrameVector3D yoSensorToDistalCoMvectorInWorld;
   private final Wrench distalMassWrench;

   private final YoFrameVector3D yoSensorForce;
   private final YoFrameVector3D yoSensorTorque;

   private final YoFrameVector3D yoSensorForceFromDistalMass;
   private final YoFrameVector3D yoSensorTorqueFromDistalMass;

   private final YoFrameVector3D yoSensorForceMassCompensated;
   private final YoFrameVector3D yoSensorTorqueMassCompensated;

   private final YoBoolean addSimulatedSensorNoise;

   public ForceSensorDistalMassCompensator(ForceSensorDefinition forceSensorDefinition, double dtForLowpassFilter, YoRegistry registry)
   {
      String sensorName = forceSensorDefinition.getSensorName();

      sensorFrame = forceSensorDefinition.getSensorFrame();

      sensorPose = new FramePose3D(world);
      yoSensorPositionInWorld = new YoFramePoint3D(sensorName + "Position", world, registry);

      distalMassCalc = new CenterOfMassCalculator(forceSensorDefinition.getRigidBody(), world);
      distalMass = new YoDouble(sensorName + "DistalMass", registry);
      lowPassSensorForceZ = new AlphaFilteredYoVariable(sensorName + "LowPassFz",
                                                        registry,
                                                        AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.0001, dtForLowpassFilter));
      distalMassForceInWorld = new YoFrameVector3D(sensorName + "DistalWeight", world, registry);
      distalCoMInWorld = new YoFramePoint3D(sensorName + "DistalCoM", world, registry);

      yoSensorToDistalCoMvectorInWorld = new YoFrameVector3D(sensorName + "ToDistalCoM", world, registry);
      distalMassWrench = new Wrench(sensorFrame, world);

      // Put sensor values in world frame since it's easy to interpret from looking at GUI
      yoSensorForce = new YoFrameVector3D(sensorName + "Force", world, registry);
      yoSensorTorque = new YoFrameVector3D(sensorName + "Torque", world, registry);

      yoSensorForceFromDistalMass = new YoFrameVector3D(sensorName + "ForceDueToDistalMass", world, registry);
      yoSensorTorqueFromDistalMass = new YoFrameVector3D(sensorName + "TorqueDueToDistalMass", world, registry);

      yoSensorForceMassCompensated = new YoFrameVector3D(sensorName + "ForceMassCompensated", world, registry);
      yoSensorTorqueMassCompensated = new YoFrameVector3D(sensorName + "TorqueMassCompensated", world, registry);

      addSimulatedSensorNoise = new YoBoolean(sensorName + "AddSimulatedNoise", registry);
      addSimulatedSensorNoise.set(false);
   }

   public double getDistalMass()
   {
      return distalMass.getDoubleValue();
   }

   public FramePoint3DReadOnly getSensorPosition()
   {
      return yoSensorPositionInWorld;
   }

   public ReferenceFrame getSensorReferenceFrame()
   {
      return sensorFrame;
   }

   public FrameVector3DReadOnly getSensorForceRaw()
   {
      return yoSensorForce;
   }

   public FrameVector3DReadOnly getSensorTorqueRaw()
   {
      return yoSensorTorque;
   }

   public FrameVector3DReadOnly getSensorForceMassCompensated()
   {
      return yoSensorForceMassCompensated;
   }

   public FrameVector3DReadOnly getSensorTorqueMassCompensated()
   {
      return yoSensorTorqueMassCompensated;
   }

   public double getSensorZForceLowPassFilteredInWorld()
   {
      return lowPassSensorForceZ.getDoubleValue();
   }

   public void update(WrenchReadOnly sensorWrench)
   {
      yoSensorForce.setMatchingFrame(sensorWrench.getLinearPart());
      yoSensorTorque.setMatchingFrame(sensorWrench.getAngularPart());

      if (addSimulatedSensorNoise.getBooleanValue())
      {
         double amp = 0.1;
         double bias = 0.25;

         yoSensorForce.add(amp * 2.0 * (Math.random() - 0.5) + bias, amp * 2.0 * (Math.random() - 0.5) + bias, amp * 2.0 * (Math.random() - 0.5) + bias);
         yoSensorTorque.add(amp * 2.0 * (Math.random() - 0.5) + bias, amp * 2.0 * (Math.random() - 0.5) + bias, amp * 2.0 * (Math.random() - 0.5) + bias);
      }

      updateSensorPosition();
      updateCenterOfMass();
      yoSensorToDistalCoMvectorInWorld.sub(distalCoMInWorld, yoSensorPositionInWorld);

      distalMassWrench.setToZero(world);
      distalMassWrench.setIncludingFrame(null, distalMassForceInWorld, new FramePoint3D(yoSensorToDistalCoMvectorInWorld));

      yoSensorForceFromDistalMass.set(distalMassWrench.getReferenceFrame(),
                                      distalMassWrench.getLinearPartX(),
                                      distalMassWrench.getLinearPartY(),
                                      distalMassWrench.getLinearPartZ());
      yoSensorTorqueFromDistalMass.set(distalMassWrench.getReferenceFrame(),
                                       distalMassWrench.getAngularPartX(),
                                       distalMassWrench.getAngularPartY(),
                                       distalMassWrench.getAngularPartZ());

      yoSensorForceMassCompensated.sub(yoSensorForce, yoSensorForceFromDistalMass);
      yoSensorTorqueMassCompensated.sub(yoSensorTorque, yoSensorTorqueFromDistalMass);
   }

   private final FramePoint3D temp = new FramePoint3D();

   private void updateSensorPosition()
   {
      sensorFrame.update();

      sensorPose.set(sensorFrame.getTransformToDesiredFrame(world));
      temp.setIncludingFrame(sensorPose.getPosition());
      yoSensorPositionInWorld.set(temp.getReferenceFrame(), temp.getX(), temp.getY(), temp.getZ());
   }

   private void updateCenterOfMass()
   {
      distalMassCalc.reset();
      distalMass.set(distalMassCalc.getTotalMass());
      distalMassForceInWorld.set(0.0, 0.0, Math.abs(GRAVITY) * distalMass.getDoubleValue());

      distalCoMInWorld.set(distalMassCalc.getCenterOfMass());

      lowPassSensorForceZ.update(yoSensorForce.getZ());
   }
}
