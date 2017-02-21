package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.commons.Conversions;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.tools.maps.EnumDoubleMap;

public class StepprAirwalk implements StepprController
{
   private enum AirwalkState
   {
      WAIT, INITIALIZE, GOTO_ZERO, AIRWALK
   }

   private final static double trajectoryTime = 10.0;

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprAirwalk");

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final YoPolynomial trajectory = new YoPolynomial("trajectory", 4, registry);

   private final EnumMap<StepprJoint, OneDoFJoint> joints = new EnumMap<>(StepprJoint.class);
   private final EnumMap<StepprJoint, PDController> controllers = new EnumMap<>(StepprJoint.class);

   private final EnumMap<StepprJoint, DoubleYoVariable> desiredOffsets = new EnumMap<>(StepprJoint.class);
   private final EnumDoubleMap<StepprJoint> initialPositions = new EnumDoubleMap<>(StepprJoint.class);

   private final BooleanYoVariable startAirwalk = new BooleanYoVariable("startAirwalk", registry);
   private final EnumYoVariable<AirwalkState> airwalkState = new EnumYoVariable<>("airwalkState", registry, AirwalkState.class);

   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableOutput", registry);

   private final DoubleYoVariable hipAmplitude = new DoubleYoVariable("hipAmplitude", registry);
   private final DoubleYoVariable hipFrequency = new DoubleYoVariable("hipFrequency", registry);

   private final DoubleYoVariable kneeAmplitude = new DoubleYoVariable("kneeAmplitude", registry);
   private final DoubleYoVariable kneeFrequency = new DoubleYoVariable("kneeFrequency", registry);

   private final DoubleYoVariable ankleAmplitude = new DoubleYoVariable("ankleAmplitude", registry);
   private final DoubleYoVariable ankleFrequency = new DoubleYoVariable("ankleFrequency", registry);

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         joints.put(joint, fullRobotModel.getOneDoFJointByName(joint.getSdfName()));
         controllers.put(joint, new PDController(joint.getSdfName(), registry));
      }

      for (StepprStandPrepSetpoints setpoint : StepprStandPrepSetpoints.values)
      {

         for (int i = 0; i < setpoint.getJoints().length; i++)
         {
            StepprJoint joint = setpoint.getJoints()[i];
            DoubleYoVariable desiredPosition = new DoubleYoVariable(setpoint.getName() + "_q_d", registry);
            desiredPosition.set(setpoint.getQ());

            if (i == 1)
            {
               desiredPosition.mul(setpoint.getReflectRight());
            }
            desiredOffsets.put(joint, desiredPosition);
            controllers.get(joint).setProportionalGain(setpoint.getKp());
            controllers.get(joint).setProportionalGain(setpoint.getKd());
            joints.get(joint).setKd(setpoint.getDamping());
         }
      }

   }

   @Override
   public void initialize(long timestamp)
   {
      startAirwalk.set(false);
      airwalkState.set(AirwalkState.WAIT);
      enableOutput.set(false);
   }

   @Override
   public void doControl(long timestamp)
   {
      double time = Conversions.nanoSecondstoSeconds(timestamp);
      switch (airwalkState.getEnumValue())
      {
      case WAIT:
         if (startAirwalk.getBooleanValue())
         {
            airwalkState.set(AirwalkState.INITIALIZE);
         }
         break;

      case INITIALIZE:
         initialTime.set(time);
         trajectory.setCubic(0.0, trajectoryTime, 0.0, 0.0, 1.0, 0.0);

         for (StepprJoint joint : StepprJoint.values)
         {
            initialPositions.put(joint, joints.get(joint).getQ());
         }
         airwalkState.set(AirwalkState.GOTO_ZERO);
         enableOutput.set(true);
         break;

      case GOTO_ZERO:
         double timeInTrajectory = MathTools.clipToMinMax(time - initialTime.getDoubleValue(), 0, trajectoryTime);
         trajectory.compute(timeInTrajectory);
         double positionScale = trajectory.getPosition();

         for (StepprJoint joint : StepprJoint.values)
         {
            double setpoint = desiredOffsets.get(joint).getDoubleValue();
            double initialPosition = initialPositions.get(joint);

            double qDesired = initialPosition + (setpoint - initialPosition) * positionScale;
            double qdDesired = 0;
            controlJoint(joint, qDesired, qdDesired);
         }

         if (timeInTrajectory > trajectoryTime)
         {
            airwalkState.set(AirwalkState.AIRWALK);
         }
         break;
      case AIRWALK:
         double sinusoidTime = time - (initialTime.getDoubleValue() + trajectoryTime);

         for (StepprJoint joint : StepprJoint.values)
         {
            double qDesired = desiredOffsets.get(joint).getDoubleValue();
            controlJoint(joint, qDesired, 0);
         }

         StepprJoint[] hips = StepprStandPrepSetpoints.HIP_Y.getJoints();
         for (int i = 0; i < hips.length; i++)
         {
            double qDesired = desiredOffsets.get(hips[i]).getDoubleValue();
            qDesired += hipAmplitude.getDoubleValue() * (i == 0 ? 1.0 : -1.0) * Math.sin(2.0 * Math.PI * hipFrequency.getDoubleValue() * sinusoidTime);
            controlJoint(hips[i], qDesired, 0);
         }
         StepprJoint[] knees = StepprStandPrepSetpoints.KNEE.getJoints();
         for (int i = 0; i < knees.length; i++)
         {
            double qDesired = desiredOffsets.get(knees[i]).getDoubleValue();
            qDesired += kneeAmplitude.getDoubleValue() * (i == 0 ? -1.0 : 1.0) * Math.sin(2.0 * Math.PI * kneeFrequency.getDoubleValue() * sinusoidTime);
            controlJoint(knees[i], qDesired, 0);
         }
         StepprJoint[] ankles = StepprStandPrepSetpoints.ANKLE_Y.getJoints();
         for (int i = 0; i < ankles.length; i++)
         {
            double qDesired = desiredOffsets.get(ankles[i]).getDoubleValue();
            qDesired += ankleAmplitude.getDoubleValue() * (i == 0 ? 1.0 : -1.0) * Math.sin(2.0 * Math.PI * ankleFrequency.getDoubleValue() * sinusoidTime);
            controlJoint(ankles[i], qDesired, 0);
         }

         break;
      }

   }

   private void controlJoint(StepprJoint joint, double qDesired, double qdDesired)
   {
      OneDoFJoint oneDoFJoint = joints.get(joint);
      PDController controller = controllers.get(joint);
      double tau = controller.compute(oneDoFJoint.getQ(), qDesired, oneDoFJoint.getQd(), qdDesired);
      oneDoFJoint.setTau(tau);
   }

   @Override
   public boolean turnOutputOn()
   {
      boolean enable = enableOutput.getBooleanValue();
      enableOutput.set(false); // Outputs only need to be enabled once, after that you can turn them off
      //      return enable;

      return false;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprAirwalk());
   }

}
