package us.ihmc.simulationToolkit.controllers;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ActualCMPComputer extends SimpleRobotController
{
   private static final boolean visibleByDefault = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final HumanoidFloatingRootJointRobot simulatedRobot;
   private final double simulateDT;
   private final double gravity;

   private final Vector3D linearMomentum = new Vector3D();
   private final Vector3D linearMomentumRate = new Vector3D();
   private final Point3D comPosition = new Point3D();
   private final Point2D comPosition2d = new Point2D();
   private final Vector2D comAcceleration = new Vector2D();
   private final Point2D cmp = new Point2D();

   private final YoDouble alpha = new YoDouble("momentumRateAlpha", registry);
   private final YoFrameVector3D yoLinearMomentum = new YoFrameVector3D("linearMomentum", worldFrame, registry);
   private final FilteredVelocityYoFrameVector momentumChange;

   private final YoFrameVector2D yoCmp = new YoFrameVector2D("actualCMP", worldFrame, registry);

   public ActualCMPComputer(boolean createViz, SimulationConstructionSet scs, HumanoidFloatingRootJointRobot simulatedRobot)
   {
      this.simulatedRobot = simulatedRobot;
      simulateDT = scs.getDT();
      gravity = simulatedRobot.getGravityZ();

      momentumChange = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector("rateOfChangeLinearMomentum", "", alpha, simulateDT, registry, yoLinearMomentum);

      if (createViz)
      {
         yoGraphicsListRegistry = new YoGraphicsListRegistry();
         YoArtifactPosition cmpViz = new YoArtifactPosition("SimulationCMP", yoCmp.getYoX(), yoCmp.getYoY(),
               GraphicType.BALL_WITH_CROSS, Color.RED , 0.005);
         cmpViz.setVisible(visibleByDefault);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), cmpViz);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      }
      else
      {
         yoGraphicsListRegistry = null;
      }
   }

   @Override
   public void doControl()
   {
      // update the linear momentum rate by numerical differentiation of the robot momentum
      simulatedRobot.getRootJoint().physics.recursiveComputeLinearMomentum(linearMomentum);
      yoLinearMomentum.set(linearMomentum);
      momentumChange.update();
      linearMomentumRate.set(momentumChange);

      // get mass and COM position from the robot
      double totalMass = simulatedRobot.computeCenterOfMass(comPosition);
      comPosition2d.set(comPosition.getX(), comPosition.getY());

      // now compute the COM acceleration
      comAcceleration.set(linearMomentumRate.getX(), linearMomentumRate.getY());
      comAcceleration.scale(1.0 / totalMass);

      // CMP = COM - 1/omega^2 * COMAcc
      double omega0 = Math.sqrt(-gravity / comPosition.getZ());
      cmp.set(comAcceleration);
      cmp.scale(- 1.0 / (omega0 * omega0));
      cmp.add(comPosition2d);

      yoCmp.set(cmp);
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }
}