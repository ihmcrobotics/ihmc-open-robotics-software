package us.ihmc.simulationToolkit.controllers;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.io.printing.PrintTools;

public class SimulateCutforceController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExternalForcePoint efpWrist;
   private final ExternalForcePoint efpHandControlFrame;
   private final Point3d handControlFramePositionInWorld;
   private final Vector3d wristToHandControlFrame;
   private final Vector3d tangentVector;
   private final Vector3d forceVector;
   private final Vector3d climbingForceVector;
   private final Vector3d xAxisVector;
   private final Vector3d tangentionalVelocity;

   private final DoubleYoVariable efpHandControlFrameVelocity;
   private final DoubleYoVariable efpForce;

   private final FloatingRootJointRobot sdfRobot;
   private final FullRobotModel fullRobotModel;
   private final RobotSide robotSide;

   private final Joint wristJoint;
   private final RigidBodyTransform transform;

   private final static double gGRAVITY = 9.81;

   //TODO estimate:
   private DoubleYoVariable quadraticForceCoeff;

   // Graphicregistry for viz
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final FramePose wristJointPose, handControlFramePose;
   private final YoFramePose yoWristJointPose, yoHandControlFramePose;

   public SimulateCutforceController(FloatingRootJointRobot robot, FullHumanoidRobotModel fullRobotModel, RobotSide robotSide, SimulationConstructionSet scs)
   {
      this.sdfRobot = robot;
      this.fullRobotModel = fullRobotModel;
      this.robotSide = robotSide;

      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      wristJoint = robot.getJoint(fullRobotModel.getHand(this.robotSide).getParentJoint().getName());
      transform = new RigidBodyTransform();
      wristToHandControlFrame = new Vector3d();
      tangentVector = new Vector3d();
      forceVector = new Vector3d();
      tangentionalVelocity = new Vector3d();
      climbingForceVector = new Vector3d();
      xAxisVector = new Vector3d(1.0, 0.0, 0.0);

      efpHandControlFrameVelocity = new DoubleYoVariable("cutforceSimulatorVelocity", registry);
      efpForce = new DoubleYoVariable("cutforceSimulatorForce", registry);

      switch (this.robotSide)
      {
      case LEFT:
         wristToHandControlFrame.set(0.0, 0.26, 0.0);
         break;
      case RIGHT:
         wristToHandControlFrame.set(0.0, -0.26, 0.0);
         break;
      default:
         PrintTools.error(this, "No robotSide assigned.");
         break;
      }

      efpWrist = new ExternalForcePoint("wrist", sdfRobot);
      efpHandControlFrame = new ExternalForcePoint("tooltip", wristToHandControlFrame, sdfRobot);
      handControlFramePositionInWorld = new Point3d();

      wristJoint.addExternalForcePoint(efpWrist);
      wristJoint.addExternalForcePoint(efpHandControlFrame);

      wristJoint.getTransformToWorld(transform);
      wristJointPose = new FramePose(HumanoidReferenceFrames.getWorldFrame(), transform);
      yoWristJointPose = new YoFramePose("wristJointPose", HumanoidReferenceFrames.getWorldFrame(), registry);
      yoWristJointPose.set(wristJointPose);
      YoGraphicCoordinateSystem yoWristCoordinateSystem = new YoGraphicCoordinateSystem("wristCoordinateSystemViz", yoWristJointPose, 0.1, YoAppearance.Red());

      wristJoint.getTransformToWorld(transform);
      transform.transform(wristToHandControlFrame, tangentVector);
      handControlFramePose = new FramePose(HumanoidReferenceFrames.getWorldFrame(), transform);
      handControlFramePose.translate(tangentVector);
      yoHandControlFramePose = new YoFramePose("handControlFrame",HumanoidReferenceFrames.getWorldFrame(), registry);
      yoHandControlFramePose.set(handControlFramePose);
      YoGraphicCoordinateSystem yoToolTip = new YoGraphicCoordinateSystem("toolTipViz", yoHandControlFramePose, 0.1, YoAppearance.Yellow());


      yoGraphicsListRegistry.registerYoGraphic("drillComViz", yoWristCoordinateSystem);
      yoGraphicsListRegistry.registerYoGraphic("drillToolTipViz", yoToolTip);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      quadraticForceCoeff = new DoubleYoVariable("quadraticForceCoeff", registry);
      quadraticForceCoeff.set(1000.0);

   }

   @Override
   public void initialize()
   {
      PrintTools.debug(this, "CutforceSimulator initialized");
      doControl();
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
      wristJoint.getTransformToWorld(transform);
      wristJointPose.setPose(transform);
      yoWristJointPose.set(wristJointPose);

      wristJoint.getTransformToWorld(transform);
      transform.transform(wristToHandControlFrame, tangentVector);
      handControlFramePose.setPose(transform);
      handControlFramePose.translate(tangentVector);
      yoHandControlFramePose.set(handControlFramePose);
      handControlFramePose.getPosition(handControlFramePositionInWorld);

      efpHandControlFrame.setPosition(handControlFramePositionInWorld);

//      efpGravity.setForce(0.0, 0.0, -gGRAVITY * MASSDRILL);
//      efpGravity.setForce(0.0, 0.0, 0.0);
//      efpWrist.setForce(exponentialCutForceModel(efpHandControlFrame));
      efpWrist.setForce(quadraticCutForceModel(efpHandControlFrame));

   }

   private Vector3d quadraticCutForceModel(ExternalForcePoint forcePoint)
   {
	   tangentVector.set(forcePoint.getVelocityVector());
	   tangentionalVelocity.set(forcePoint.getVelocityVector());

	   if(this.sdfRobot.getTime() >=13 && this.sdfRobot.getTime() <= 17)
	   {
		   quadraticForceCoeff.set(10000.0);
	   }
	   else
	   {
		   quadraticForceCoeff.set(1000.0);
	   }
	     if(tangentVector.length() != 0.0 && forcePoint.getPositionPoint().getZ() > 0.75 && forcePoint.getPositionPoint().getX() > 0.5)
	     {
	    	tangentionalVelocity.setX(0.0);
	    	tangentVector.setX(0.0);
	        tangentVector.normalize();
	        climbingForceVector.cross(tangentVector, xAxisVector);

	        tangentVector.scale(-1.0);
	        tangentVector.scale(quadraticForceCoeff.getDoubleValue() * Math.pow(forcePoint.getVelocityVector().length(), 2));
	        climbingForceVector.scale(tangentionalVelocity.length() * 100.0);

	        efpHandControlFrameVelocity.set(tangentionalVelocity.length());

	        forceVector.set(tangentVector);
//	        forceVector.add(climbingForceVector);
	        efpForce.set(forceVector.length());
	        return forceVector;

	     }
	     else
	     {
	        return tangentVector;
	     }
   }


   private Vector3d exponentialCutForceModel(ExternalForcePoint forcePoint)
   {
	   tangentVector.set(forcePoint.getVelocityVector());
	   tangentionalVelocity.set(forcePoint.getVelocityVector());

	     if(tangentVector.length() != 0.0)
	     {
	    	tangentionalVelocity.setX(0.0);
	    	tangentVector.setX(0.0);
	        tangentVector.normalize();
	        tangentVector.scale(-1.0);
	        tangentVector.scale(90.0 * (Math.exp(1.0 *tangentionalVelocity.length()) - 1.0));
	        efpHandControlFrameVelocity.set(tangentionalVelocity.length());
	        return tangentVector;
	     }
	     else
	     {
	        return tangentVector;
	     }
   }
}
