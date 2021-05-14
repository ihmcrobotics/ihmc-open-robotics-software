package us.ihmc.exampleSimulations.footPathRunners;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootPathRunnerController implements RobotController
{
   private final FootPathRunnerRobot robot;
   private final double dt;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble metaCenterRadius = new YoDouble("metaCenterRadius", registry);
   private final YoDouble comBelowMetaCenterDistance = new YoDouble("comBelowMetaCenterDistance", registry);
   private final YoDouble comInFrontOfLegsDistance = new YoDouble("comInFrontOfLegsDistance", registry);

   private final YoDouble nominalVelocity = new YoDouble("nominalVelocity", registry);

   private final YoDouble footExcursionRange = new YoDouble("footExcursionRange", registry);
   private final YoDouble footExcursionDuration = new YoDouble("footExcursionDuration", registry);
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);

   private final YoDouble swingLiftAmount = new YoDouble("swingLiftAmount", registry);

   private final SideDependentList<YoDouble> cycleTimes;
   private final SideDependentList<YoBoolean> footIsSwingings;

   public FootPathRunnerController(FootPathRunnerRobot robot, double dt)
   {
      this.robot = robot;
      this.dt = dt;

      nominalVelocity.set(20.0);

      metaCenterRadius.set(1.6);
      comBelowMetaCenterDistance.set(0.48);
      comInFrontOfLegsDistance.set(0.25);

      footExcursionRange.set(Math.PI * 0.5);
      swingLiftAmount.set(0.5);

      double estimatedDuration = (footExcursionRange.getValue() * metaCenterRadius.getValue()) / nominalVelocity.getValue();

      swingDuration.set(estimatedDuration);
      footExcursionDuration.set(estimatedDuration);

      cycleTimes = new SideDependentList<YoDouble>();
      footIsSwingings = new SideDependentList<YoBoolean>();

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble cycleTime = new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "CycleTime", registry);
         YoBoolean footIsSwinging = new YoBoolean(robotSide.getCamelCaseNameForStartOfExpression() + "FootIsSwinging", registry);

         cycleTimes.put(robotSide, cycleTime);
         footIsSwingings.put(robotSide, footIsSwinging);
      }

      cycleTimes.get(RobotSide.LEFT).set(0.0);
      cycleTimes.get(RobotSide.RIGHT).set((swingDuration.getValue() + footExcursionDuration.getValue()) / 2.0);
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
   public void doControl()
   {
      robot.updateReferenceFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble cycleTime = cycleTimes.get(robotSide);

         cycleTime.add(dt);
         if (cycleTime.getValue() > footExcursionDuration.getValue() + swingDuration.getValue())
         {
            cycleTime.set(0.0);
         }

         Pose3D desiredFootInBodyFrame = getDesiredFootPoseInBodyFrame(robotSide);

         robot.setDesiredFootLocation(robotSide, desiredFootInBodyFrame);
         robot.controlFootLocation(robotSide);
      }
   }

   public Pose3D getDesiredFootPoseInBodyFrame(RobotSide robotSide)
   {
      YoDouble cycleTime = cycleTimes.get(robotSide);
      YoBoolean footIsSwinging = footIsSwingings.get(robotSide);

      if (cycleTime.getValue() > footExcursionDuration.getValue())
      {
         footIsSwinging.set(true);
      }
      else
      {
         footIsSwinging.set(false);
      }

      Pose3D desiredFootInBodyFrame = new Pose3D();

      if (!footIsSwinging.getValue())
      {
         double percent = cycleTime.getValue() / footExcursionDuration.getValue();

         double theta = -footExcursionRange.getValue() / 2.0 + percent * footExcursionRange.getValue();
         double x = -comInFrontOfLegsDistance.getValue() - metaCenterRadius.getValue() * Math.sin(theta);
         double z = comBelowMetaCenterDistance.getValue() - metaCenterRadius.getValue() * Math.cos(theta);
         double y = 0.0;

         double pitch = theta;
         double roll = 0.0;

         desiredFootInBodyFrame.getPosition().set(x, y, z);
         desiredFootInBodyFrame.getOrientation().setYawPitchRoll(0.0, pitch, roll);
      }
      else
      {
         double percent = (cycleTime.getValue() - footExcursionDuration.getValue()) / swingDuration.getValue();

         double theta = -footExcursionRange.getValue() / 2.0 + percent * footExcursionRange.getValue();
         theta = -theta;
         double x = -metaCenterRadius.getValue() * Math.sin(theta);
         double z = comBelowMetaCenterDistance.getValue() - metaCenterRadius.getValue() * Math.cos(theta)
               + swingLiftAmount.getValue() * Math.sin(percent * Math.PI);
         double y = 0.0;

         double pitch = theta;
         double roll = 0.0;

         desiredFootInBodyFrame.getPosition().set(x, y, z);
         desiredFootInBodyFrame.getOrientation().setYawPitchRoll(0.0, pitch, roll);
      }
      return desiredFootInBodyFrame;
   }

}
