package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorquesInterface;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.math.trajectories.YoMinimumJerkTrajectory;


public class KneeExtensionControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("KneeExtension");

   private final DoubleYoVariable extraKneeTorque;
   private final DoubleYoVariable desiredKneeAngle;
   private final DoubleYoVariable kp_KneeExtension;
   private final DoubleYoVariable kp_KneeBending;
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
      kp_KneeBending = new DoubleYoVariable("kp_KneeBending", registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }
   
   public void setupParametersForR2()
   {
      loadedBentKnee.set(0.3); //0.3); //0.07); //0.35); //0.7); //0.5);
      extendedKnee.set(0.0);
      bendTime.set(0.2);
      extendTime.set(0.2);

      kp_KneeExtension.set(200); // 500.0);
   }
   
   public void setupParametersForM2V2()
   {
      loadedBentKnee.set(0.3); //0.07); //0.35); //0.7); //0.5);
      extendedKnee.set(0.0);
      bendTime.set(0.01); //0.03);
      extendTime.set(0.3);

      kp_KneeExtension.set(25.0); //50.0);
      kp_KneeBending.set(75.0); //25.0);
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

   public void breakKneeForDownhillSlopes(RobotSide supportSide)
   {
      double time = processedSensors.getTime();
      yoMinimumJerkTrajectory.setParams(processedSensors.getKneeAngle(supportSide), 0.0, 0.0, loadedBentKnee.getDoubleValue()+0.05, 0.0,
                                        0.0, time, time + bendTime.getDoubleValue()-0.05);
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
      
      //TODO: Don't extend the knee if it'll tip you over backward...
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
         extraKneeTorque.set(kneeAngleError * kp_KneeBending.getDoubleValue());
         legTorquesToPackForStanceSide.addKneeTorque(extraKneeTorque.getDoubleValue());
      }
   }





}

