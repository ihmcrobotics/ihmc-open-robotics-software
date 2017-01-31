package us.ihmc.simulationconstructionset.util.virtualHoist;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;

public class VirtualHoist implements RobotController
{
   private ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<ExternalForcePoint>();

   private final YoVariableRegistry registry = new YoVariableRegistry("VirtualHoist");

   private final BooleanYoVariable hoistOn = new BooleanYoVariable("hoistOn", registry);
   private final BooleanYoVariable hoistUp = new BooleanYoVariable("hoistUp", registry);
   private final BooleanYoVariable hoistDown = new BooleanYoVariable("hoistDown", registry);

   private final List<DoubleYoVariable> cableLengths = new ArrayList<>();
   private final DoubleYoVariable physicalCableLength = new DoubleYoVariable("hoistPhysicalCableLength", registry);

   private final List<DoubleYoVariable> cableForceMagnitudes = new ArrayList<>();

   private final DoubleYoVariable hoistUpDownSpeed = new DoubleYoVariable("hoistUpDownSpeed", registry);

   private final DoubleYoVariable hoistStiffness = new DoubleYoVariable("hoistStiffness", registry);
   private final DoubleYoVariable hoistDamping = new DoubleYoVariable("hoistDamping", registry);

   private final YoFramePoint teepeeLocation = new YoFramePoint("teepeeLocation", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable q_x, q_y, q_z;

   private final double updateDT;

   public VirtualHoist(Joint jointToAttachHoist, Robot robot, ArrayList<Vector3d> hoistPointPositions, double updateDT)
   {
      this.updateDT = updateDT;

      for (int i = 0; i < hoistPointPositions.size(); i++)
      {
         Vector3d hoistPointPosition = hoistPointPositions.get(i);
         ExternalForcePoint externalForcePoint = new ExternalForcePoint("ef_hoist" + i, hoistPointPosition, robot.getRobotsYoVariableRegistry());
         externalForcePoints.add(externalForcePoint);
         jointToAttachHoist.addExternalForcePoint(externalForcePoint);

         cableLengths.add(new DoubleYoVariable("hoistCableLength" + i, registry));
         cableForceMagnitudes.add(new DoubleYoVariable("hoistCableForceMagnitude" + i, registry));
      }

      physicalCableLength.set(0.5);

      // Initial gains and teepee location:
      hoistStiffness.set(5000.0);
      hoistDamping.set(1000.0);

      hoistUpDownSpeed.set(0.08);

      turnHoistOff();
      hoistUp.set(false);
      hoistDown.set(false);

      q_x = (DoubleYoVariable) robot.getVariable("q_x");
      q_y = (DoubleYoVariable) robot.getVariable("q_y");
      q_z = (DoubleYoVariable) robot.getVariable("q_z");

      teepeeLocation.set(0.0, 0.0, 1.25);
   }

   public void setTeepeeLocation(Point3d teepeeLocation)
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
      setTeepeeLocation(new Point3d(q_x.getDoubleValue(), q_y.getDoubleValue(), q_z.getDoubleValue() + 0.5));
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

      Point3d teepeePosition = teepeeLocation.getPoint3dCopy();
      Point3d pointPosition = new Point3d();
      Vector3d forceVector = new Vector3d();
      Vector3d velocityVector = new Vector3d();

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
