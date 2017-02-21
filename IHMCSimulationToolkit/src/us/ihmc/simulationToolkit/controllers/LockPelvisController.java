package us.ihmc.simulationToolkit.controllers;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LockPelvisController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ArrayList<ExternalForcePoint> externalForcePoints = new ArrayList<>();
   private final ArrayList<Vector3D> efp_offsetFromRootJoint = new ArrayList<>();
   private final double dx = 0.5, dy = 0.5, dz = 0.0*1.0;

   private final ArrayList<Vector3D> initialPositions = new ArrayList<>();

   private final DoubleYoVariable holdPelvisKp = new DoubleYoVariable("holdPelvisKp", registry);
   private final DoubleYoVariable holdPelvisKv = new DoubleYoVariable("holdPelvisKv", registry);
   private final DoubleYoVariable desiredHeight = new DoubleYoVariable("desiredHeight", registry);
   private final double robotMass, robotWeight;

   private final FloatingRootJointRobot robot;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ArrayList<YoGraphicPosition> efp_positionViz = new ArrayList<>();

   public LockPelvisController(FloatingRootJointRobot robot, SimulationConstructionSet scs, FullRobotModel fullRobotModel, double desiredHeight)
   {
      this.robot = robot;
      robotMass = robot.computeCenterOfMass(new Point3D());
      robotWeight = robotMass * Math.abs(robot.getGravityZ());
      this.desiredHeight.set(desiredHeight);

      Joint jointToAddExternalForcePoints;
      try
      {
         //            String lastSpineJointName = fullRobotModel.getChest().getParentJoint().getName();
         jointToAddExternalForcePoints = robot.getJoint(fullRobotModel.getPelvis().getParentJoint().getName());
      }
      catch (NullPointerException e)
      {
         System.err.println("No chest or spine found. Stack trace:");
         e.printStackTrace();

         jointToAddExternalForcePoints = robot.getRootJoint();
      }

      holdPelvisKp.set(5000.0);
      holdPelvisKv.set(GainCalculator.computeDampingForSecondOrderSystem(robotMass, holdPelvisKp.getDoubleValue(), 0.6));

      efp_offsetFromRootJoint.add(new Vector3D(dx, dy, dz));
      efp_offsetFromRootJoint.add(new Vector3D(dx, -dy, dz));
      efp_offsetFromRootJoint.add(new Vector3D(-dx, dy, dz));
      efp_offsetFromRootJoint.add(new Vector3D(-dx, -dy, dz));

      for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
      {
         initialPositions.add(new Vector3D());

         String linkName = jointToAddExternalForcePoints.getLink().getName();
         ExternalForcePoint efp = new ExternalForcePoint("efp_" + linkName + "_" + String.valueOf(i) + "_", efp_offsetFromRootJoint.get(i),
               robot.getRobotsYoVariableRegistry());
         externalForcePoints.add(efp);
         jointToAddExternalForcePoints.addExternalForcePoint(efp);

         efp_positionViz.add(new YoGraphicPosition(efp.getName(), efp.getYoPosition(), 0.05, YoAppearance.Red()));
      }

      yoGraphicsListRegistry.registerYoGraphics("EFP", efp_positionViz);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
   }

   @Override
   public void initialize()
   {
      robot.update();
      for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
      {
         externalForcePoints.get(i).getYoPosition().get(initialPositions.get(i));
         desiredHeight.add(initialPositions.get(i).getZ() / initialPositions.size());
         efp_positionViz.get(i).update();
      }

      for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
         initialPositions.get(i).setZ(desiredHeight.getDoubleValue());

      doControl();
   }

   private final Vector3D proportionalTerm = new Vector3D();
   private final Vector3D derivativeTerm = new Vector3D();
   private final Vector3D pdControlOutput = new Vector3D();

   @Override
   public void doControl()
   {
      for (int i = 0; i < efp_offsetFromRootJoint.size(); i++)
      {
         initialPositions.get(i).setZ(desiredHeight.getDoubleValue());

         ExternalForcePoint efp = externalForcePoints.get(i);
         efp.getYoPosition().get(proportionalTerm);
         proportionalTerm.sub(initialPositions.get(i));
         proportionalTerm.scale(-holdPelvisKp.getDoubleValue());
//         proportionalTerm.setZ(Math.max(proportionalTerm.getZ(), 0.0));

         efp.getYoVelocity().get(derivativeTerm);
         derivativeTerm.scale(-holdPelvisKv.getDoubleValue());

         pdControlOutput.add(proportionalTerm, derivativeTerm);

         efp.setForce(pdControlOutput);
         efp.getYoForce().getYoZ().add(robotWeight / efp_offsetFromRootJoint.size());

         efp_positionViz.get(i).update();
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
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
