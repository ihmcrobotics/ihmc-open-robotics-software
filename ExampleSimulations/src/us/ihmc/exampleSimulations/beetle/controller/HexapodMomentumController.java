package us.ihmc.exampleSimulations.beetle.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.StraightLineCartesianTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;

public class HexapodMomentumController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoFrameVector yoLinearMomentumRateOfChange;
   private final YoFrameVector yoAngularMomentumRateOfChange;
   
   private final FrameVector linearMomentumRateOfChange = new FrameVector();
   private final FrameVector angularMomentumRateOfChange = new FrameVector();
   private final Vector3D linearMomentumWeight = new Vector3D(0.8, 0.8, 0.8);
   private final Vector3D angularMomentumWeight = new Vector3D(0.01, 0.01, 0.01);
   
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(6);
   private final HexapodReferenceFrames referenceFrames;
   private final FramePoint desiredComPosition = new FramePoint();
   private final StraightLineCartesianTrajectoryGenerator trajectoryGenerator;
   private final DoubleYoVariable yoTime;
   private final CenterOfMassJacobian comJacobian;
   private final YoFramePoint yoCurrentCenterOfMassPosition;
   private final YoFramePoint yoCurrentCenterOfFeetPosition;
   private final FramePoint currentCenterOfMassPosition = new FramePoint();
   private final FrameVector currentCenterOfMassVelocity = new FrameVector();
   private final FrameVector initialAcceleration = new FrameVector();
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   
   private double dt;

   public HexapodMomentumController(String prefix, HexapodReferenceFrames referenceFrames, FullRobotModel fullRobotModel, double dt, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.dt = dt;
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      yoTime = new DoubleYoVariable(prefix + "yoTime", registry);
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
      
      momentumRateCommand.setLinearMomentumRateOfChange(linearMomentumRateOfChange);
      momentumRateCommand.setAngularMomentumRateOfChange(angularMomentumRateOfChange);
      momentumRateCommand.setSelectionMatrix(selectionMatrix);
      momentumRateCommand.setWeights(angularMomentumWeight.getX(), angularMomentumWeight.getY(), angularMomentumWeight.getZ(), linearMomentumWeight.getX(),
            linearMomentumWeight.getY(), linearMomentumWeight.getZ());
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }
}
