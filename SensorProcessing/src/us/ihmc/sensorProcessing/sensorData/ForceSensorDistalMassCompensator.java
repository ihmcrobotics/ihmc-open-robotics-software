package us.ihmc.sensorProcessing.sensorData;

import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVectorInMultipleFrames;

public class ForceSensorDistalMassCompensator
{
   private final CenterOfMassCalculator distalMassCalc;
   private final DoubleYoVariable distalMass;
   private final FrameVector distalWeightInWorld;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame sensorFrame;

   private final YoFrameVectorInMultipleFrames yoSensorForce;
   private final YoFrameVectorInMultipleFrames yoSensorTorque;
   private final YoFrameVectorInMultipleFrames yoSensorForceFromDistalMass;
   private final YoFrameVectorInMultipleFrames yoSensorToHandCoMvector;
   private final YoFrameVectorInMultipleFrames yoSensorTorqueFromDistalMass;

   private final YoFrameVector yoSensorForceMassCompensated;
   private final YoFrameVector yoSensorTorqueMassCompensated;
   
   public ForceSensorDistalMassCompensator(ForceSensorDefinition forceSensorDefinition, YoVariableRegistry registry)
   {
      String sensorName = forceSensorDefinition.getSensorName();
      
      InverseDynamicsJoint parentJointOfSensorBody = forceSensorDefinition.getRigidBody().getParentJoint();
      sensorFrame = forceSensorDefinition.getSensorFrame();

      distalMassCalc = new CenterOfMassCalculator(ScrewTools.computeRigidBodiesAfterThisJoint(parentJointOfSensorBody), sensorFrame);
      distalMass = new DoubleYoVariable(sensorName + "DistalMass", registry);
      distalWeightInWorld = new FrameVector(world);

      ReferenceFrame[] worldAndSensorFrames = new ReferenceFrame[] { world, sensorFrame };

      yoSensorForce = new YoFrameVectorInMultipleFrames(sensorName + "Force", registry, worldAndSensorFrames);
      yoSensorTorque = new YoFrameVectorInMultipleFrames(sensorName + "Torque", registry, worldAndSensorFrames);

      yoSensorForceFromDistalMass = new YoFrameVectorInMultipleFrames(sensorName + "ForceDueToDistalMass", registry, worldAndSensorFrames);
      yoSensorToHandCoMvector = new YoFrameVectorInMultipleFrames(sensorName + "ToDistalCoM", registry, worldAndSensorFrames);
      yoSensorTorqueFromDistalMass = new YoFrameVectorInMultipleFrames(sensorName + "TorqueDueToDistalMass", registry, worldAndSensorFrames);

      yoSensorForceMassCompensated = new YoFrameVector(sensorName + "ForceMassCompensated", sensorFrame, registry);
      yoSensorTorqueMassCompensated = new YoFrameVector(sensorName + "TorqueMassCompensated", sensorFrame, registry);
   }
   
   Wrench sensorWrench = new Wrench();

   public void update(ForceSensorData forceSensorData)
   {
      forceSensorData.packWrench(sensorWrench);
      update(sensorWrench);
   }
   
   
   public void update(Wrench sensorWrench)
   {
      ReferenceFrame wristWrenchFrame = sensorWrench.getExpressedInFrame();
      yoSensorForce.set(wristWrenchFrame, sensorWrench.getLinearPartX(), sensorWrench.getLinearPartY(), sensorWrench.getLinearPartZ());
      yoSensorTorque.set(wristWrenchFrame, sensorWrench.getAngularPartX(), sensorWrench.getAngularPartY(), sensorWrench.getAngularPartZ());
      
      updateReferenceFramesAndCenterOfMass();
      computeSensorForceDueToWeightOfHand();
      computeSensorTorqueDueToWeightOfHand();
      
      yoSensorForceMassCompensated.sub(yoSensorForce, yoSensorForceFromDistalMass);
      yoSensorTorqueMassCompensated.sub(yoSensorTorque, yoSensorTorqueFromDistalMass);
   }
   
   public double getDistalMass()
   {
      return distalMass.getDoubleValue();
   }
   
   public FrameVector getSensorForceRaw()
   {
      return yoSensorForce.getFrameTuple();
   }
   
   public FrameVector getSensorTorqueRaw()
   {
      return yoSensorTorque.getFrameTuple();
   }
   
   public FrameVector getSensorForceMassCompensated()
   {
      return yoSensorForceMassCompensated.getFrameTuple();
   }
   
   public FrameVector getSensorTorqueMassCompensated()
   {
      return yoSensorTorqueMassCompensated.getFrameTuple();
   }
   
   private final FramePoint temp = new FramePoint();

   private void updateReferenceFramesAndCenterOfMass()
   {
      sensorFrame.update();

      distalMassCalc.compute();
      distalMass.set(distalMassCalc.getTotalMass());
      distalWeightInWorld.set(0.0, 0.0, 9.81 * distalMass.getDoubleValue());

      yoSensorToHandCoMvector.changeFrame(distalMassCalc.getDesiredFrame());
      distalMassCalc.getCenterOfMass(temp);
      yoSensorToHandCoMvector.set(temp.getReferenceFrame(), temp.getX(), temp.getY(), temp.getZ());
   }
   
   private void computeSensorForceDueToWeightOfHand()
   {
      yoSensorForceFromDistalMass.changeFrame(world);
      yoSensorForceFromDistalMass.set(world, 0.0, 0.0, 9.81 * distalMass.getDoubleValue()); // USE +9.81 since measured force is equal and opposite

      yoSensorForce.changeFrame(sensorFrame);
      yoSensorForceFromDistalMass.changeFrame(sensorFrame);
   }

   private void computeSensorTorqueDueToWeightOfHand()
   {
      yoSensorToHandCoMvector.changeFrame(distalWeightInWorld.getReferenceFrame());
      yoSensorTorqueFromDistalMass.changeFrame(distalWeightInWorld.getReferenceFrame());

      FrameVector wristTorqueDueToHandMass = yoSensorTorqueFromDistalMass.getFrameTuple();
      wristTorqueDueToHandMass.cross(yoSensorToHandCoMvector.getFrameTuple(), distalWeightInWorld);

      yoSensorTorqueFromDistalMass.set(wristTorqueDueToHandMass.getReferenceFrame(), wristTorqueDueToHandMass.getX(), wristTorqueDueToHandMass.getY(),
            wristTorqueDueToHandMass.getZ());

      yoSensorTorque.changeFrame(sensorFrame);
      yoSensorTorqueFromDistalMass.changeFrame(sensorFrame);
   }
}
