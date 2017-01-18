package us.ihmc.simulationToolkit.controllers;

import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class ActualCMPComputer extends SimpleRobotController
{
   private static final boolean visibleByDefault = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final HumanoidFloatingRootJointRobot simulatedRobot;
   private final double simulateDT;
   private final double gravity;

   private final Vector3d linearMomentum = new Vector3d();
   private final Vector3d linearMomentumRate = new Vector3d();
   private final Point3d comPosition = new Point3d();
   private final Point2d comPosition2d = new Point2d();
   private final Vector2d comAcceleration = new Vector2d();
   private final Point2d cmp = new Point2d();

   private final DoubleYoVariable alpha = new DoubleYoVariable("momentumRateAlpha", registry);
   private final YoFrameVector yoLinearMomentum = new YoFrameVector("linearMomentum", worldFrame, registry);
   private final FilteredVelocityYoFrameVector momentumChange;

   private final YoFrameVector2d yoCmp = new YoFrameVector2d("actualCMP", worldFrame, registry);

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
      momentumChange.get(linearMomentumRate);

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