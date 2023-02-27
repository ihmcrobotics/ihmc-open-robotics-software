package us.ihmc.simulationToolkit.controllers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.simulationConstructionSetTools.util.perturbance.GroundContactPointsSlipper;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootMotionOnTouchdownSlipper implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final List<Controller> subControllers = new ArrayList<>();
   private final SideDependentList<List<GroundContactPoint>> groundContactPointsMap = new SideDependentList<>();
   private final HashMap<GroundContactPoint, YoFramePoint3D> groundContactTouchdownMap = new HashMap<>();

   private final YoDouble time = new YoDouble("time", registry);
   private final SideDependentList<YoDouble> timesAtContact = new SideDependentList<>();
   private final SideDependentList<YoBoolean> footIsInContacts = new SideDependentList<>();

   private final YoDouble slipDuration;
   private final YoFrameVector3D translationMagnitudes;
   private final YoFrameYawPitchRoll rotationMagnitudesYawPitchRoll;

   private final double deltaT;

   public FootMotionOnTouchdownSlipper(Robot robot, SideDependentList<String> footNames, double deltaT)
   {
      String name = "FootSlip";

      this.deltaT = deltaT;

      for (RobotSide robotSide : RobotSide.values)
      {
         List<GroundContactPoint> groundContactPoints = robot.getRigidBody(footNames.get(robotSide)).getParentJoint().getAuxialiryData()
                                                             .getGroundContactPoints();
         groundContactPointsMap.put(robotSide, groundContactPoints);
         YoDouble timeAtContact = new YoDouble(robotSide.getCamelCaseName() + "TimeAtContact", registry);

         for (GroundContactPoint groundContactPoint : groundContactPoints)
         {
            groundContactTouchdownMap.put(groundContactPoint,
                                          new YoFramePoint3D(groundContactPoint.getName() + "AtTouchdown", ReferenceFrame.getWorldFrame(), getYoRegistry()));
         }

         YoBoolean footIsInContact = new YoBoolean(robotSide.getCamelCaseName() + "FootIsInContact", registry);
         footIsInContact.addListener((v) ->
         {
            if (footIsInContact.getBooleanValue())
            {
               timeAtContact.set(time.getDoubleValue());
               for (GroundContactPoint groundContactPoint : groundContactPoints)
               {
                  groundContactTouchdownMap.get(groundContactPoint).set(groundContactPoint.getTouchdownPose().getPosition());
               }
            }
         });

         timesAtContact.put(robotSide, timeAtContact);
         footIsInContacts.put(robotSide, footIsInContact);
      }

      translationMagnitudes = new YoFrameVector3D(name + "TranslationMagnitude", ReferenceFrame.getWorldFrame(), registry);
      slipDuration = new YoDouble(name + "SilpDuration", registry);
      rotationMagnitudesYawPitchRoll = new YoFrameYawPitchRoll(name + "RotationRate", ReferenceFrame.getWorldFrame(), registry);

      GroundContactPointsSlipper leftSlipper = new GroundContactPointsSlipper("left");
      GroundContactPointsSlipper rightSlipper = new GroundContactPointsSlipper("right");

      subControllers.add(leftSlipper);
      subControllers.add(rightSlipper);
   }

   public void setTranslationMagnitudes(double[] translation)
   {
      this.translationMagnitudes.set(translation[0], translation[1], translation[2]);
   }

   public void setRotationMagnitudesYawPitchRoll(double[] rotationYawPitchRoll)
   {
      this.rotationMagnitudesYawPitchRoll.setYawPitchRoll(rotationYawPitchRoll[0], rotationYawPitchRoll[1], rotationYawPitchRoll[2]);
   }

   public void setSlipDuration(double duration)
   {
      slipDuration.set(duration);
   }

   @Override
   public void doControl()
   {
      subControllers.forEach(Controller::doControl);
      time.add(deltaT);

      for (RobotSide robotSide : RobotSide.values())
      {
         updateFootContactState(robotSide);
         updateSlipping(robotSide);
      }
   }

   private void updateSlipping(RobotSide robotSide)
   {
      if (footIsInContacts.get(robotSide).getBooleanValue())
      {
         double timeInState = time.getDoubleValue() - timesAtContact.get(robotSide).getDoubleValue();
         if (timeInState < slipDuration.getDoubleValue())
         {
            double alpha = MathTools.clamp(timeInState / slipDuration.getDoubleValue(), 0.0, 1.0);
            List<GroundContactPoint> groundContactPointsToSlip = groundContactPointsMap.get(robotSide);

            Vector3D translationAmount = new Vector3D(translationMagnitudes);
            Vector3D rotationAmount = new Vector3D(rotationMagnitudesYawPitchRoll.getYaw(),
                                                   rotationMagnitudesYawPitchRoll.getPitch(),
                                                   rotationMagnitudesYawPitchRoll.getRoll());
            translationAmount.scale(alpha);
            rotationAmount.scale(alpha);

            Point3D touchdownCoM = computeTouchdownCoM(groundContactPointsToSlip);
            touchdownCoM.add(translationAmount);

            for (int i = 0; i < groundContactPointsToSlip.size(); i++)
            {
               GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

               Point3D touchdownLocation = new Point3D(groundContactTouchdownMap.get(groundContactPointToSlip));
               touchdownLocation.add(translationAmount);
               //               touchdownLocation.sub(touchdownCoM);

               //               RotationMatrix deltaRotation = new RotationMatrix(rotationAmount);
               //               deltaRotation.transform(touchdownLocation);

               //               touchdownLocation.add(touchdownCoM);
               groundContactPointToSlip.getTouchdownPose().getPosition().set(touchdownLocation);
               groundContactPointToSlip.getInContact().set(true);
            }
         }
      }
   }

   private Point3D computeTouchdownCoM(List<GroundContactPoint> groundContactPointsToSlip)
   {
      int touchdownCount = 0;
      Point3D touchdownCoM = new Point3D();

      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         touchdownCoM.add(groundContactTouchdownMap.get(groundContactPointToSlip));
         touchdownCount++;
      }

      touchdownCoM.scale(1.0 / touchdownCount);
      return touchdownCoM;
   }

   private void updateFootContactState(RobotSide robotSide)
   {
      for (GroundContactPoint groundContactPoint : groundContactPointsMap.get(robotSide))
      {
         if (groundContactPoint.getInContact().getValue())
         {
            footIsInContacts.get(robotSide).set(true);
            return;
         }
      }

      footIsInContacts.get(robotSide).set(false);
   }
}
