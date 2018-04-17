package us.ihmc.exampleSimulations.fallingBox;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GroundContactPointBasedEstimator implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final GroundContactPointBasedWrenchCalculator wrenchCalculator;

   private final YoWrench wrench;

   public GroundContactPointBasedEstimator(Robot robot)
   {
      registry = new YoVariableRegistry(robot.getName() + "_registry");

      Joint joint = robot.getJoint("bodyJoint");
      ArrayList<GroundContactPoint> groundContactPoints = robot.getAllGroundContactPoints();
      wrenchCalculator = new GroundContactPointBasedWrenchCalculator(robot.getName() + "_ft", groundContactPoints, joint, new RigidBodyTransform(), registry);

      wrench = new YoWrench(robot.getName() + "_wrench", worldFrame, worldFrame, registry);
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      wrenchCalculator.calculate();

      DenseMatrix64F wrenchMatrix = wrenchCalculator.getWrench();
      Wrench wrench = new Wrench(worldFrame, worldFrame, wrenchMatrix);

      this.wrench.set(wrench);
   }
}
