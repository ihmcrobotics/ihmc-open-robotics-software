package us.ihmc.exampleSimulations.beetle.controller;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.StraightLineCartesianTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class HexapodMomentumController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoFrameVector yoLinearMomentumRateOfChange;
   private final YoFrameVector yoAngularMomentumRateOfChange;
   
   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D angularMomentumRateOfChange = new FrameVector3D();
   private final Vector3D linearMomentumWeight = new Vector3D(0.8, 0.8, 0.8);
   private final Vector3D angularMomentumWeight = new Vector3D(0.01, 0.01, 0.01);
   
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final HexapodReferenceFrames referenceFrames;
   private final FramePoint3D desiredComPosition = new FramePoint3D();
   private final StraightLineCartesianTrajectoryGenerator trajectoryGenerator;
   private final YoDouble yoTime;
   private final CenterOfMassJacobian comJacobian;
   private final YoFramePoint yoCurrentCenterOfMassPosition;
   private final YoFramePoint yoCurrentCenterOfFeetPosition;
   private final FramePoint3D currentCenterOfMassPosition = new FramePoint3D();
   private final FrameVector3D currentCenterOfMassVelocity = new FrameVector3D();
   private final FrameVector3D initialAcceleration = new FrameVector3D();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   
   private double dt;

   public HexapodMomentumController(String prefix, HexapodReferenceFrames referenceFrames, FullRobotModel fullRobotModel, double dt, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.dt = dt;
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      yoTime = new YoDouble(prefix + "yoTime", registry);
      yoLinearMomentumRateOfChange = new YoFrameVector(prefix + "desiredLinearMomentumRateOfChange", ReferenceFrame.getWorldFrame(), registry);  
      yoAngularMomentumRateOfChange = new YoFrameVector(prefix + "desiredAngularMomentumRateOfChange", ReferenceFrame.getWorldFrame(), registry);
      trajectoryGenerator = new StraightLineCartesianTrajectoryGenerator("comTrajectoryGenerator", ReferenceFrame.getWorldFrame(), 3.0, 10.0, yoTime, registry);
      yoCurrentCenterOfMassPosition = new YoFramePoint(prefix + "centerOfMassPosition", ReferenceFrame.getWorldFrame(), registry);
      yoCurrentCenterOfFeetPosition = new YoFramePoint(prefix + "centerOfFeetPosition", ReferenceFrame.getWorldFrame(), registry);
      
      YoGraphicPosition comPositionGraphic = new YoGraphicPosition(prefix + "centerOfMassPosition", yoCurrentCenterOfMassPosition, 0.02, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerYoGraphic("comPositionGraphic", comPositionGraphic);
      yoGraphicsListRegistry.registerArtifact("comPositionGraphic", comPositionGraphic.createArtifact());
      
      YoGraphicPosition centerOfFeetGraphic = new YoGraphicPosition(prefix + "centerOfFeetGraphic", yoCurrentCenterOfFeetPosition, 0.02, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("centerOfFeetGraphic", centerOfFeetGraphic);
      yoGraphicsListRegistry.registerArtifact("centerOfFeetGraphic", centerOfFeetGraphic.createArtifact());
      
      parentRegistry.addChild(registry);
   }
   
   public void initialize()
   {
      
   }

   public void doControl()
   {
      yoTime.add(dt);
      
      currentCenterOfMassPosition.setToZero(referenceFrames.getCenterOfMassFrame());
      currentCenterOfMassPosition.changeFrame(ReferenceFrame.getWorldFrame());
      yoCurrentCenterOfMassPosition.set(currentCenterOfMassPosition);
      
      comJacobian.compute();
      comJacobian.getCenterOfMassVelocity(currentCenterOfMassVelocity);
      currentCenterOfMassVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      
      desiredComPosition.setToZero(referenceFrames.getCenterOfFeetFrame());
      desiredComPosition.changeFrame(referenceFrames.getCenterOfMassFrame());
      desiredComPosition.setZ(0.0);
      desiredComPosition.changeFrame(ReferenceFrame.getWorldFrame());
      yoCurrentCenterOfFeetPosition.set(desiredComPosition);
      
      trajectoryGenerator.initialize(currentCenterOfMassPosition, currentCenterOfMassVelocity, initialAcceleration, desiredComPosition, currentCenterOfMassVelocity);
      trajectoryGenerator.computeNextTick(desiredPosition, desiredVelocity, linearMomentumRateOfChange, dt);
      
      yoLinearMomentumRateOfChange.set(linearMomentumRateOfChange);
      yoAngularMomentumRateOfChange.set(angularMomentumRateOfChange);
      
      momentumRateCommand.setMomentumRate(angularMomentumRateOfChange, linearMomentumRateOfChange);
      momentumRateCommand.setSelectionMatrix(selectionMatrix);
      momentumRateCommand.setWeights(angularMomentumWeight.getX(), angularMomentumWeight.getY(), angularMomentumWeight.getZ(), linearMomentumWeight.getX(),
            linearMomentumWeight.getY(), linearMomentumWeight.getZ());
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }
}
