package us.ihmc.simulationConstructionSetTools.util.perturbance;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

//Used to make ground contact points slip a delta.
public class GroundContactPointsSlipper implements Controller
{
   private final YoRegistry registry;

   private final ArrayList<GroundContactPoint> groundContactPointsToSlip;
   private final YoFrameVector3D slipAmount;
   private final YoFrameYawPitchRoll slipRotation;

   private final YoDouble percentToSlipPerTick;
   private final YoBoolean doSlip;

   public GroundContactPointsSlipper(String registryPrefix)
   {
      registry = new YoRegistry(registryPrefix + getClass().getSimpleName());

      groundContactPointsToSlip = new ArrayList<GroundContactPoint>();
      slipAmount = new YoFrameVector3D("slipAmount", SimulationSession.DEFAULT_INERTIAL_FRAME, registry);
      slipRotation = new YoFrameYawPitchRoll("slipRotation", SimulationSession.DEFAULT_INERTIAL_FRAME, registry);

      percentToSlipPerTick = new YoDouble("percentToSlipPerTick", registry);
      doSlip = new YoBoolean("doSlip", registry);
   }

   public void addGroundContactPoints(List<GroundContactPoint> footGroundContactPoints)
   {
      for (GroundContactPoint groundContactPoint : footGroundContactPoints)
      {
         addGroundContactPoint(groundContactPoint);
      }
   }

   public void setGroundContactPoints(List<GroundContactPoint> footGroundContactPoints)
   {
      this.groundContactPointsToSlip.clear();

      for (GroundContactPoint groundContactPoint : footGroundContactPoints)
      {
         addGroundContactPoint(groundContactPoint);
      }
   }

   public void addGroundContactPoint(GroundContactPoint groundContactPoint)
   {
      this.groundContactPointsToSlip.add(groundContactPoint);
   }

   public void addGroundContactPoint(Robot robot, String groundContactPointName)
   {
      List<GroundContactPoint> allGroundContactPoints = robot.getAllJoints().stream().flatMap(j -> j.getAuxialiryData().getGroundContactPoints().stream())
                                                             .collect(Collectors.toList());

      for (GroundContactPoint groundContactPoint : allGroundContactPoints)
      {
         if (groundContactPoint.getName().equals(groundContactPointName))
         {
            this.addGroundContactPoint(groundContactPoint);

            return;
         }
      }
   }

   public void setDoSlip(boolean doSlip)
   {
      this.doSlip.set(doSlip);
   }

   public void setPercentToSlipPerTick(double percentToSlipPerTick)
   {
      this.percentToSlipPerTick.set(percentToSlipPerTick);
   }

   public void setSlipTranslation(Vector3DReadOnly slipAmount)
   {
      this.slipAmount.set(slipAmount);
   }

   public void setSlipRotationYawPitchRoll(double[] yawPitchRoll)
   {
      this.slipRotation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setSlipRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      this.slipRotation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setSlipRotationEulerAngles(Vector3DReadOnly eulerAngles)
   {
      this.slipRotation.setEuler(eulerAngles);
   }

   public boolean isDoneSlipping()
   {
      boolean translationalSlipDone = slipAmount.lengthSquared() < 0.0001 * 0.0001;

      Vector3D eulerAngles = new Vector3D();
      slipRotation.getEuler(eulerAngles);
      boolean rotationalSlipDone = eulerAngles.lengthSquared() < 0.001 * 0.001;

      return translationalSlipDone & rotationalSlipDone;
   }

   public void slipALittle(double percentOfDelta)
   {
      if (percentOfDelta < 0.0)
         return;
      if (percentOfDelta > 1.0)
         return;

      applyTranslationalSlip(percentOfDelta);
      applyRotationalSlip(percentOfDelta);
   }

   private void applyTranslationalSlip(double percentOfDelta)
   {
      FrameVector3D slipDelta = new FrameVector3D(slipAmount);
      slipDelta.scale(percentOfDelta);
      slipAmount.sub(slipDelta);

      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         boolean touchedDown = (groundContactPointToSlip.getInContact().getValue());

         if (touchedDown)
         {
            groundContactPointToSlip.getTouchdownPose().getPosition().add(slipDelta);
         }
      }
   }

   private void applyRotationalSlip(double percentOfDelta)
   {
      Quaternion identity = new Quaternion();
      Quaternion desired = new Quaternion(slipRotation);
      Quaternion delta = new Quaternion();

      delta.interpolate(identity, desired, percentOfDelta);

      desired.interpolate(identity, desired, 1.0 - percentOfDelta);
      slipRotation.set(desired);

      Point3D touchdownCoM = computeTouchdownCoM();
      RotationMatrix deltaRotation = new RotationMatrix(delta);


      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         boolean touchedDown = (groundContactPointToSlip.getInContact().getValue());

         if (touchedDown)
         {
            YoFramePoint3D touchdownPosition = groundContactPointToSlip.getTouchdownPose().getPosition();
            touchdownPosition.sub(touchdownCoM);
            deltaRotation.transform(touchdownPosition);
            touchdownPosition.add(touchdownCoM);
         }
      }
   }

   private Point3D computeTouchdownCoM()
   {
      int touchdownCount = 0;
      Point3D touchdownCoM = new Point3D();

      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         boolean touchedDown = (groundContactPointToSlip.getInContact().getValue());
         if (touchedDown)
         {
            touchdownCoM.add(groundContactPointToSlip.getTouchdownPose().getPosition());
            touchdownCount++;
         }
      }

      touchdownCoM.scale(1.0 / touchdownCount);
      return touchdownCoM;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public void doControl()
   {
      if (doSlip.getBooleanValue())
      {
         slipALittle(percentToSlipPerTick.getDoubleValue());
      }
   }

}
