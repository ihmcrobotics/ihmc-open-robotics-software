package us.ihmc.exampleSimulations.fallingBox;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GroundContactPointBasedEstimator implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final GroundContactPointBasedWrenchCalculator wrenchCalculator;

   private final YoFixedFrameWrench wrench;

   public GroundContactPointBasedEstimator(Robot robot)
   {
      registry = new YoRegistry(robot.getName() + "_registry");

      Joint joint = robot.getJoint("bodyJoint");
      List<GroundContactPoint> groundContactPoints = robot.getAllGroundContactPoints();
      wrenchCalculator = new GroundContactPointBasedWrenchCalculator(robot.getName() + "_ft", groundContactPoints, joint, new RigidBodyTransform(), registry);

      wrench = new YoFixedFrameWrench(robot.getName() + "_wrench", worldFrame, worldFrame, registry);
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
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      wrenchCalculator.calculate();

      DMatrixRMaj wrenchMatrix = wrenchCalculator.getWrench();
      Wrench wrench = new Wrench(worldFrame, worldFrame, wrenchMatrix);

      this.wrench.set(wrench);
   }
}
