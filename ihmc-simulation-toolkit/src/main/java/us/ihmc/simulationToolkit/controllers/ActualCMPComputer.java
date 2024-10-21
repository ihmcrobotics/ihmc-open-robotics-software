package us.ihmc.simulationToolkit.controllers;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.yoVariables.euclid.filters.FilteredFiniteDifferenceYoFrameVector3D;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoDouble;

public class ActualCMPComputer extends SimpleRobotController
{
   private static final boolean visibleByDefault = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final FloatingRootJointRobot simulatedRobot;
   private final double simulateDT;
   private final double gravity;

   private final Vector3D linearMomentum = new Vector3D();
   private final Vector3D linearMomentumRate = new Vector3D();
   private final Vector3D angularMomentum = new Vector3D();
   private final Point3D comPosition = new Point3D();
   private final Vector3D comVector = new Vector3D();
   private final Vector3D angularMomentumAboutCenterOfMass = new Vector3D();
   private final Point2D comPosition2d = new Point2D();
   private final Vector3D comAcceleration = new Vector3D();
   private final Point2D cmp = new Point2D();

   private final YoDouble alpha = new YoDouble("simMomentumRateAlpha", registry);
   private final YoFrameVector3D yoLinearMomentum = new YoFrameVector3D("simLinearMomentum", worldFrame, registry);
   private final YoFrameVector3D yoAngularMomentum = new YoFrameVector3D("simAngularMomentum", worldFrame, registry);
   private final FilteredFiniteDifferenceYoFrameVector3D linearMomentumRateOfChange;
   private final FilteredFiniteDifferenceYoFrameVector3D angularMomentumRateOfChange;

   private final YoFrameVector2D yoCmp = new YoFrameVector2D("simActualCMP", worldFrame, registry);

   public ActualCMPComputer(boolean createViz, SimulationConstructionSet scs, FloatingRootJointRobot simulatedRobot)
   {
      this.simulatedRobot = simulatedRobot;
      simulateDT = scs.getDT();
      gravity = simulatedRobot.getGravityZ();

      linearMomentumRateOfChange = new FilteredFiniteDifferenceYoFrameVector3D("SimRateOfChangeLinearMomentum", "", alpha, simulateDT, registry, yoLinearMomentum);
      angularMomentumRateOfChange = new FilteredFiniteDifferenceYoFrameVector3D("SimRateOfChangeAngularMomentum", "", alpha, simulateDT, registry, yoAngularMomentum);

      if (createViz)
      {
         yoGraphicsListRegistry = new YoGraphicsListRegistry();
         YoArtifactPosition cmpViz = new YoArtifactPosition("SimulationCMP", yoCmp.getYoX(), yoCmp.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.005);
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
      // update the linear momentum rate by numerical differentiation of the robot linear momentum
      simulatedRobot.getRootJoint().physics.recursiveComputeLinearMomentum(linearMomentum);
      yoLinearMomentum.set(linearMomentum);
      linearMomentumRateOfChange.update();
      linearMomentumRate.set(linearMomentumRateOfChange);

      // get mass and COM position from the robot
      double totalMass = simulatedRobot.computeCenterOfMass(comPosition);
      comPosition2d.set(comPosition.getX(), comPosition.getY());

      // now compute the COM acceleration
      comAcceleration.set(linearMomentumRate);
      comAcceleration.scale(1.0 / totalMass);

      // CMP = COMxy - (z/Fz)*Fxy
      cmp.set(comAcceleration);
      double z = comPosition.getZ();
      double normalizedFz = -gravity + comAcceleration.getZ();
      cmp.scale(-z / normalizedFz);
      cmp.add(comPosition2d);

      yoCmp.set(cmp);

      // update the angular momentum rate by numerical differentiation of the robot angular momentum
      simulatedRobot.getRootJoint().physics.recursiveComputeAngularMomentum(angularMomentum);

      // Transform angular momentum to be about the Center of Mass:
      comVector.set(comPosition);
      angularMomentumAboutCenterOfMass.cross(linearMomentum, comVector);
      angularMomentumAboutCenterOfMass.add(angularMomentum);

      yoAngularMomentum.set(angularMomentumAboutCenterOfMass);
      angularMomentumRateOfChange.update();
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }
}