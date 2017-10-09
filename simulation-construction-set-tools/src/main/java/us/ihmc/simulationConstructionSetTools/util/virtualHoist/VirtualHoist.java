package us.ihmc.simulationConstructionSetTools.util.virtualHoist;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

public class VirtualHoist implements RobotController
{
   private ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<ExternalForcePoint>();

   private final YoVariableRegistry registry = new YoVariableRegistry("VirtualHoist");

   private final YoBoolean hoistOn = new YoBoolean("hoistOn", registry);
   private final YoBoolean hoistUp = new YoBoolean("hoistUp", registry);
   private final YoBoolean hoistDown = new YoBoolean("hoistDown", registry);

   private final List<YoDouble> cableLengths = new ArrayList<>();
   private final YoDouble physicalCableLength = new YoDouble("hoistPhysicalCableLength", registry);

   private final List<YoDouble> cableForceMagnitudes = new ArrayList<>();

   private final YoDouble hoistUpDownSpeed = new YoDouble("hoistUpDownSpeed", registry);

   private final YoDouble hoistStiffness = new YoDouble("hoistStiffness", registry);
   private final YoDouble hoistDamping = new YoDouble("hoistDamping", registry);

   private final YoFramePoint teepeeLocation = new YoFramePoint("teepeeLocation", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble q_x, q_y, q_z;

   private final double updateDT;

   public VirtualHoist(Joint jointToAttachHoist, Robot robot, ArrayList<Vector3D> hoistPointPositions, double updateDT)
   {
      this.updateDT = updateDT;

      for (int i = 0; i < hoistPointPositions.size(); i++)
      {
         Vector3D hoistPointPosition = hoistPointPositions.get(i);
         ExternalForcePoint externalForcePoint = new ExternalForcePoint("ef_hoist" + i, hoistPointPosition, robot.getRobotsYoVariableRegistry());
         externalForcePoints.add(externalForcePoint);
         jointToAttachHoist.addExternalForcePoint(externalForcePoint);

         cableLengths.add(new YoDouble("hoistCableLength" + i, registry));
         cableForceMagnitudes.add(new YoDouble("hoistCableForceMagnitude" + i, registry));
      }

      physicalCableLength.set(0.5);

      // Initial gains and teepee location:
      hoistStiffness.set(5000.0);
      hoistDamping.set(1000.0);

      hoistUpDownSpeed.set(0.08);

      turnHoistOff();
      hoistUp.set(false);
      hoistDown.set(false);

      q_x = (YoDouble) robot.getVariable("q_x");
      q_y = (YoDouble) robot.getVariable("q_y");
      q_z = (YoDouble) robot.getVariable("q_z");

      teepeeLocation.set(0.0, 0.0, 1.25);
   }

   public void setTeepeeLocation(Point3D teepeeLocation)
   {
      this.teepeeLocation.set(teepeeLocation);
   }

   public void turnHoistOff()
   {
      hoistOn.set(false);
   }

   public void setHoistStiffness(double hoistStiffness)
   {
      this.hoistStiffness.set(hoistStiffness);
   }

   public void setHoistDamping(double hoistDamping)
   {
      this.hoistDamping.set(hoistDamping);
   }

   /**
    * Turns the hoist on and sets the pivot point
    */
   public void turnHoistOn()
   {
      hoistOn.set(true);
   }

   public void moveHoistUp()
   {
      turnHoistOn();

      hoistUp.set(true);
      hoistDown.set(false);
   }

   public void moveHoistDown()
   {
      turnHoistOn();

      hoistUp.set(false);
      hoistDown.set(true);
   }

   public void stopMovingHoist()
   {
      hoistUp.set(false);
      hoistDown.set(false);
   }

   public void putHoistOverRobot()
   {
      setTeepeeLocation(new Point3D(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue() + 0.5));
   }

   public double getHoistHeight()
   {
      return teepeeLocation.getZ();
   }

   @Override
   public void doControl()
   {
      // For each external force point, figure out the force vector and apply it:

      if (!hoistOn.getBooleanValue())
      {
         for (ExternalForcePoint externalForcePoint : externalForcePoints)
         {
            externalForcePoint.setForce(0.0, 0.0, 0.0);
         }

         return;
      }

      if (hoistUp.getBooleanValue())
      {
         teepeeLocation.getYoZ().add(hoistUpDownSpeed.getDoubleValue() * updateDT);
      }

      if (hoistDown.getBooleanValue())
      {
         teepeeLocation.getYoZ().sub(hoistUpDownSpeed.getDoubleValue() * updateDT);
      }

      Point3D teepeePosition = teepeeLocation.getPoint3dCopy();
      Point3D pointPosition = new Point3D();
      Vector3D forceVector = new Vector3D();
      Vector3D velocityVector = new Vector3D();

      for (int i = 0; i < externalForcePoints.size(); i++)
      {
         ExternalForcePoint externalForcePoint = externalForcePoints.get(i);
         pointPosition.set(externalForcePoint.getX(), externalForcePoint.getY(), externalForcePoint.getZ());

         double cableLength = teepeePosition.distance(pointPosition);
         cableLengths.get(i).set(cableLength);

         if (cableLength < physicalCableLength.getDoubleValue())
         {
            externalForcePoint.setForce(0.0, 0.0, 0.0);
         }

         else
         {
            forceVector.sub(teepeePosition, pointPosition);
            double delta = forceVector.length() - physicalCableLength.getDoubleValue();
            double springForce = hoistStiffness.getDoubleValue() * delta;

            velocityVector.set(externalForcePoint.getXVelocity(), externalForcePoint.getYVelocity(), externalForcePoint.getZVelocity());
            forceVector.normalize();

            double damperForce = -velocityVector.dot(forceVector) * hoistDamping.getDoubleValue();
            double totalForce = springForce + damperForce;

            if (totalForce < 0.0)
               totalForce = 0.0;

            cableForceMagnitudes.get(i).set(totalForce);

            forceVector.scale(totalForce);

            externalForcePoint.setForce(forceVector);

         }
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "VirtualHoist";
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

}
