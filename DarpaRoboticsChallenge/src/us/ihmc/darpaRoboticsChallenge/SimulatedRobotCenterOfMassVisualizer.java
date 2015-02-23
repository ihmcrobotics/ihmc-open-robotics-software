package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class SimulatedRobotCenterOfMassVisualizer implements RobotController
{
   private final Robot robot;

   private final YoVariableRegistry registry = new YoVariableRegistry("ExactCoMCalcualtor");
   private final YoFramePoint exactCenterOfMassPosition = new YoFramePoint("exactCenterOfMassPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector exactCenterOfMassVelocity = new YoFrameVector("exactCenterOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final Point3d tempCenterOfMassPoint = new Point3d();
   private final Vector3d tempCenterOfMassVelocity = new Vector3d();
   private final Vector3d tempAngularMomentum = new Vector3d();


   public SimulatedRobotCenterOfMassVisualizer(Robot robot)
   {
      this.robot = robot;
   }


   @Override
   public void doControl()
   {
      double mass = robot.computeCOMMomentum(tempCenterOfMassPoint, tempCenterOfMassVelocity, tempAngularMomentum);

      exactCenterOfMassPosition.set(tempCenterOfMassPoint);
      tempCenterOfMassVelocity.scale(1.0 / mass);
      exactCenterOfMassVelocity.set(tempCenterOfMassVelocity);
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
      return getClass().getSimpleName();
   }


   @Override
   public String getDescription()
   {
      return getName();
   }
}
