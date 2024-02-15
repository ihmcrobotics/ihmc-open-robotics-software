package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingRobot
{
   private final SideDependentList<SimPrismaticJoint> kneeJoints;
   private final SideDependentList<SimRevoluteJoint> hipJoints;

   private final SideDependentList<YoDouble> legLengths = new SideDependentList<YoDouble>();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<ReferenceFrame> footFrames = new SideDependentList<>();

   private final DoubleProvider time;
   private final ReferenceFrame worldFrame;

   private final ReferenceFrame centerOfMassFrame;

   public BWCPlanarWalkingRobot(Robot robot, DoubleProvider time)
   {
      this.time = time;
      robot.getFloatingRootJoint().setJointPosition(new Vector3D(0.0, 0.0, 0.75));

      worldFrame = robot.getInertialFrame();
      kneeJoints = new SideDependentList<>();
      hipJoints = new SideDependentList<>();

      // FIXME use the center of mass jacobian calculator for this.
      centerOfMassFrame = robot.getJoint(BWCPlanarWalkingRobotDefinition.baseJointName).getFrameAfterJoint();

      for (RobotSide robotSide : RobotSide.values)
      {
         SimPrismaticJoint kneeJoint = (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.kneeNames.get(robotSide));
         SimRevoluteJoint hipJoint = (SimRevoluteJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.hipNames.get(robotSide));
         kneeJoints.put(robotSide, kneeJoint);
         hipJoints.put(robotSide, hipJoint);

         YoDouble legLength = new YoDouble(robotSide.getLowerCaseName() + "LegLength", registry);
         legLengths.put(robotSide, legLength);

         Vector3D footTranslationFromKnee = new Vector3D();
         footTranslationFromKnee.setZ(-BWCPlanarWalkingRobotDefinition.shinLength / 2.0);
         ReferenceFrame footFrame = new FixedReferenceFrame(robotSide.getLowerCaseName() + "FootFrame", kneeJoint.getFrameAfterJoint(), footTranslationFromKnee);
         footFrames.put(robotSide, footFrame);
      }
      kneeJoints.get(RobotSide.LEFT).setQ(0.25);
   }

   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public DoubleProvider getTime()
   {
      return time;
   }

   public double getLegLength(RobotSide robotSide)
   {
      return legLengths.get(robotSide).getDoubleValue();
   }

   public ReferenceFrame getFootFrame(RobotSide robotSide)
   {
      return footFrames.get(robotSide);
   }

   public SimPrismaticJoint getKneeJoint(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide);
   }

   public SimRevoluteJoint getHipJoint(RobotSide robotSide)
   {
      return hipJoints.get(robotSide);
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         // update the current leg length
         double restingLegLength = (BWCPlanarWalkingRobotDefinition.thighLength + BWCPlanarWalkingRobotDefinition.shinLength) / 2.0;
         double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();

         legLengths.get(robotSide).set(currentLegLength);

         footFrames.get(robotSide).update();
      }
   }
}
