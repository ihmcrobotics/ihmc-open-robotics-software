package us.ihmc.darpaRoboticsChallenge.controllers;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class SimulateCutforceController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final ExternalForcePoint efpGravity;
   private final ExternalForcePoint efpToolTip;
   private final Vector3d comOffset;
   private final Vector3d toolTipOffset;
   private final Vector3d tempVector;
   
   private final SDFRobot sdfRobot;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final RobotSide robotSide;
   
   private final Joint wristJoint;
   private final RigidBodyTransform transform;
   
   private final static double gGRAVITY = 9.81;
   
   //TODO measure:
   private final static double MASSDRILL = 0.3;
   private final static double HANDOFFSET = 0.26;
   private final static double COMOFFSET = 0.00;
   private final static double TOOLTIPOFFSET = 0.18;
   
   //TODO estimate:
   private DoubleYoVariable quadraticForcecoeff;
   
   // Graphicregistry for viz
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final FramePose comPose, toolTipPose;
   private final YoFramePose yoComPose, yoToolTipPose;
   
   public SimulateCutforceController(SDFRobot robot, SDFFullRobotModel sdfFullRobotModel, RobotSide robotSide, SimulationConstructionSet scs)
   {
      this.sdfRobot = robot;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.robotSide = robotSide;
      
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      wristJoint = robot.getJoint(sdfFullRobotModel.getHand(this.robotSide).getParentJoint().getName());
//      System.out.println(wristJoint);
      
      transform = new RigidBodyTransform();
      comOffset = new Vector3d();
      toolTipOffset = new Vector3d();
      tempVector = new Vector3d();
      switch (this.robotSide)
      {
      case LEFT:
         comOffset.set(-COMOFFSET, HANDOFFSET, 0.0);
         toolTipOffset.set(-TOOLTIPOFFSET, HANDOFFSET, 0.0);
         break;
      case RIGHT:
         comOffset.set(-COMOFFSET, -HANDOFFSET, 0.0);
         toolTipOffset.set(-TOOLTIPOFFSET,-HANDOFFSET, 0.0);
         break;
      default:
         PrintTools.error(this, "No robotSide assigned.");
         break;
      }

      efpGravity = new ExternalForcePoint("Gravity", comOffset, robot);
      efpToolTip = new ExternalForcePoint("Cutmodel", toolTipOffset, sdfRobot);
      
      wristJoint.addExternalForcePoint(efpGravity);
      wristJoint.addExternalForcePoint(efpToolTip);
      
      wristJoint.getTransformToWorld(transform);
      transform.transform(comOffset, tempVector);
      comPose = new FramePose(ReferenceFrames.getWorldFrame(), transform);
      comPose.translate(tempVector);
      yoComPose = new YoFramePose("yoComPose", ReferenceFrames.getWorldFrame(), registry);
      yoComPose.set(comPose);
      YoGraphicCoordinateSystem yoCom = new YoGraphicCoordinateSystem("drillComViz", yoComPose, 0.1, YoAppearance.Red());
      
      wristJoint.getTransformToWorld(transform);
      transform.transform(toolTipOffset, tempVector);
      toolTipPose = new FramePose(ReferenceFrames.getWorldFrame(), transform);
      toolTipPose.translate(tempVector);
      yoToolTipPose = new YoFramePose("yoToolTipPose",ReferenceFrames.getWorldFrame(), registry);
      yoToolTipPose.set(toolTipPose);
      YoGraphicCoordinateSystem yoToolTip = new YoGraphicCoordinateSystem("toolTipViz", yoToolTipPose, 0.1, YoAppearance.Yellow());
      
      
      yoGraphicsListRegistry.registerYoGraphic("drillComViz", yoCom);
      yoGraphicsListRegistry.registerYoGraphic("drillToolTipViz", yoToolTip);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      
      //
      quadraticForcecoeff = new DoubleYoVariable("quadraticForceCoeff", registry);
      quadraticForcecoeff.set(5000.0);
            
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
      transform.transform(comOffset, tempVector);
      comPose.setPose(transform);
      comPose.translate(tempVector);
      yoComPose.set(comPose);
      
      wristJoint.getTransformToWorld(transform);
      transform.transform(toolTipOffset, tempVector);
      toolTipPose.setPose(transform);
      toolTipPose.translate(tempVector);
      yoToolTipPose.set(toolTipPose);
      
      
      
      efpGravity.setForce(0.0, 0.0, -gGRAVITY * MASSDRILL);
//      efpGravity.setForce(0.0, 0.0, 0.0);
      efpToolTip.setForce(quadraticCutForceModel(efpToolTip.getVelocityVector()));
   }
   
   private Vector3d quadraticCutForceModel(Vector3d toolTipVelocity)
   {
     tempVector.set(toolTipVelocity);
     
     if(tempVector.length() != 0.0)
     {
        tempVector.normalize();
        tempVector.scale(-1.0);
        
        tempVector.scale(quadraticForcecoeff.getDoubleValue() * Math.pow(toolTipVelocity.length(), 2));
        return tempVector;
     }
     else
     {
        return tempVector;
     }
   }
   
   

}
