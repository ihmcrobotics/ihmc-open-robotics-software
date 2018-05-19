package us.ihmc.exampleSimulations.fallingBox;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.CollisionShapeBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeBasedEstimator implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final CollisionShapeBasedWrenchCalculator wrenchCalculator;

   private final YoWrench wrench;

   public CollisionShapeBasedEstimator(Robot robot)
   {
      registry = new YoVariableRegistry(robot.getName() + "_registry");

      Joint joint = robot.getJoint("bodyJoint");
      ArrayList<ExternalForcePoint> contactingExternalForcePoints = robot.getAllExternalForcePoints();

      wrenchCalculator = new CollisionShapeBasedWrenchCalculator(robot.getName() + "_ft", contactingExternalForcePoints, joint, new RigidBodyTransform(),
                                                                 registry);

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
