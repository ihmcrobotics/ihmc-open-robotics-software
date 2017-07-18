package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCentroidalMomentumPivot;

import us.ihmc.commonWalkingControlModules.desiredFootStep.CenterOfMassTrajectoryHandler;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PrecomputedICPPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   private final YoFramePoint yoDesiredCMPPosition = new YoFramePoint(name + "DesiredCMPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoDesiredCoMPosition = new YoFramePoint(name + "DesiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoDesiredICPPosition = new YoFramePoint(name + "DesiredICPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector yoDesiredICPVelocity = new YoFrameVector(name + "DesiredICPVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble omega0 = new YoDouble(name + "Omega0", registry);
   
   private final FramePoint desiredICPPosition = new FramePoint();
   private final FrameVector desiredICPVelocity = new FrameVector();
   
   private final CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler;
   
   public PrecomputedICPPlanner(CenterOfMassTrajectoryHandler centerOfMassTrajectoryHandler, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.centerOfMassTrajectoryHandler = centerOfMassTrajectoryHandler;
      parentRegistry.addChild(registry);
      
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
      
      YoGraphicPosition desiredICPPositionGraphic = new YoGraphicPosition("precomputedICPTrajPosition", yoDesiredICPPosition, 0.008,
            YoAppearance.Blue(), GraphicType.SOLID_BALL);
      yoGraphicsList.add(desiredICPPositionGraphic);
      artifactList.add(desiredICPPositionGraphic.createArtifact());
      
      YoGraphicPosition desiredCenterOfMassPositionViz = new YoGraphicPosition("precomputedCoMTrajPosition", yoDesiredCoMPosition, 0.004, YoAppearance.YellowGreen(),
            GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(desiredCenterOfMassPositionViz);
      artifactList.add(desiredCenterOfMassPositionViz.createArtifact());

      YoGraphicPosition desiredCMPPositionViz = new YoGraphicPosition("precomputedCMPTrajPosition", yoDesiredCMPPosition, 0.008, YoAppearance.CornflowerBlue(),
            GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsList.add(desiredCMPPositionViz);
      artifactList.add(desiredCMPPositionViz.createArtifact());
      
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }
   
   public void compute(double time)
   {
      double omega0 = this.omega0.getDoubleValue();
      centerOfMassTrajectoryHandler.packDesiredICPAtTime(time, omega0, desiredICPPosition, desiredICPVelocity);
      computeDesiredCentroidalMomentumPivot(desiredICPPosition, desiredICPVelocity, omega0, yoDesiredCMPPosition);
      
      yoDesiredICPPosition.set(desiredICPPosition);
      yoDesiredICPVelocity.set(desiredICPVelocity);
   }
   
   public boolean isWithinInterval(double time)
   {
      return centerOfMassTrajectoryHandler.isWithinInterval(time);
   }
   
   /**
    * Intrinsic robot parameter.
    * <p>
    * Correspond the natural frequency response of the robot when modeled as an inverted pendulum:
    * {@code omega0 = Math.sqrt(g / z0)}, where {@code g} is equal to the magnitude of the gravity,
    * and {@code z0} is the constant center of mass height of the robot with respect to is feet.
    * </p>
    *
    * @param omega0 the robot's natural frequency.
    */
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }
   

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   public void getDesiredCapturePointPosition(FramePoint2d desiredCapturePointPositionToPack)
   {
      yoDesiredICPPosition.getFrameTuple2dIncludingFrame(desiredCapturePointPositionToPack);
   }
   
   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   public void getDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocityToPack)
   {
      yoDesiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }
   
   /**
    * Gets the current CMP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCentroidalMomentumPivotPositionToPack the current CMP position. Modified.
    */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack)
   {
      yoDesiredCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }
   
}
