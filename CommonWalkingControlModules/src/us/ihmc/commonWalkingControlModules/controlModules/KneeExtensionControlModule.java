package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorquesInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
//import us.ihmc.projectM.R2Sim02.partNamesAndTorques.LegJointName;
//import us.ihmc.projectM.R2Sim02.partNamesAndTorques.LegTorques;
//import us.ihmc.projectM.R2Sim02.sensors.ProcessedSensors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.YoMinimumJerkTrajectory;

public class KneeExtensionControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("KneeExtension");

   private final DoubleYoVariable extraKneeTorque;
   private final DoubleYoVariable desiredKneeAngle;
   private final DoubleYoVariable kp_KneeExtension;
   private final DoubleYoVariable loadedBentKnee, extendedKnee;
   private final DoubleYoVariable bendTime;
   private final DoubleYoVariable extendTime;

   private final YoMinimumJerkTrajectory yoMinimumJerkTrajectory;

   public KneeExtensionControlModule(ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      yoMinimumJerkTrajectory = new YoMinimumJerkTrajectory("kneeTraj", registry);

      extraKneeTorque = new DoubleYoVariable("extraKneeTorque", registry);
      desiredKneeAngle = new DoubleYoVariable("desiredKneeAngle", registry);
      loadedBentKnee = new DoubleYoVariable("loadedBentKnee", registry);
      extendedKnee = new DoubleYoVariable("extendedKnee", registry);

      bendTime = new DoubleYoVariable("bendTime", registry);
      extendTime = new DoubleYoVariable("extendTime", registry);

      kp_KneeExtension = new DoubleYoVariable("kp_KneeExtension", registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }
   
   public void setupParametersForR2()
   {
      loadedBentKnee.set(0.3); //0.07); //0.35); //0.7); //0.5);
      extendedKnee.set(0.0);
      bendTime.set(0.2);
      extendTime.set(0.2);

      kp_KneeExtension.set(500.0);
   }
   
   public void setupParametersForM2V2()
   {
      loadedBentKnee.set(0.3); //0.07); //0.35); //0.7); //0.5);
      extendedKnee.set(0.0);
      bendTime.set(0.3);
      extendTime.set(0.3);

      kp_KneeExtension.set(50.0);
   }

   public void doLoadingControl(LegTorquesInterface legTorquesToPackForStanceSide)
   {
      doKneeBendingControl(legTorquesToPackForStanceSide);
   }

   public void doEarlyStanceKneeExtensionControl(LegTorquesInterface legTorquesToPackForStanceSide)
   {
      doKneeExtensionControl(legTorquesToPackForStanceSide);
   }

   public void doLateStanceKneeExtensionControl(LegTorquesInterface legTorquesToPackForStanceSide)
   {
      doKneeExtensionControl(legTorquesToPackForStanceSide);
   }


   private void doKneeBendingControl(LegTorquesInterface legTorquesToPackForStanceSide)
   {
      computeDesiredKneeAngleFromTrajectory();
      setExtraKneeTorqueForBending(desiredKneeAngle.getDoubleValue(), legTorquesToPackForStanceSide);
   }
   
   private void doKneeExtensionControl(LegTorquesInterface legTorquesToPackForStanceSide)
   {
      computeDesiredKneeAngleFromTrajectory();
      setExtraKneeTorqueForExtending(desiredKneeAngle.getDoubleValue(), legTorquesToPackForStanceSide);
   }

   private void computeDesiredKneeAngleFromTrajectory()
   {
      yoMinimumJerkTrajectory.computeTrajectory(processedSensors.getTime());
      desiredKneeAngle.set(yoMinimumJerkTrajectory.getPosition());
   }


   public void doTransitionIntoLoading(RobotSide supportSide)
   {
      double time = processedSensors.getTime();
      yoMinimumJerkTrajectory.setParams(processedSensors.getKneeAngle(supportSide), 0.0, 0.0, loadedBentKnee.getDoubleValue(), 0.0,
                                        0.0, time, time + bendTime.getDoubleValue());
   }

   public void doTransitionIntoStance(RobotSide supportSide)
   {
      double time = processedSensors.getTime();
      yoMinimumJerkTrajectory.setParams(processedSensors.getKneeAngle(supportSide), 0.0, 0.0, extendedKnee.getDoubleValue(), 0.0,
                                        0.0, time, time + extendTime.getDoubleValue());
   }



   private void setExtraKneeTorqueForExtending(double desiredKnee, LegTorquesInterface legTorquesToPackForStanceSide)
   {
      RobotSide supportSide = legTorquesToPackForStanceSide.getRobotSide();

      double kneeAngleError = desiredKnee - processedSensors.getKneeAngle(supportSide);
      
      if (kneeAngleError < 0.0)
      {
         extraKneeTorque.set(kneeAngleError * kp_KneeExtension.getDoubleValue());
         legTorquesToPackForStanceSide.addKneeTorque(extraKneeTorque.getDoubleValue());
      }
   }
   
   private void setExtraKneeTorqueForBending(double desiredKnee, LegTorquesInterface legTorquesToPackForStanceSide)
   {
      RobotSide supportSide = legTorquesToPackForStanceSide.getRobotSide();

      double kneeAngleError = desiredKnee - processedSensors.getKneeAngle(supportSide);

      if (kneeAngleError > 0.0)
      {
         extraKneeTorque.set(kneeAngleError * kp_KneeExtension.getDoubleValue());
         legTorquesToPackForStanceSide.addKneeTorque(extraKneeTorque.getDoubleValue());
      }
   }




}

