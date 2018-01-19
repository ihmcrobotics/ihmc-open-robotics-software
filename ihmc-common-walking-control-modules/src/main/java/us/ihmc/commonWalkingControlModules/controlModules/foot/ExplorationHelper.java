package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Class provides a CenterOfPressureCommand for the QP that is used to explore the foothold by shifting the
 * desired CoP to the corners of the foothold.
 */
public class ExplorationHelper
{
   private final YoBoolean footholdExplorationActive;
   private final ExplorationParameters explorationParameters;
   private final ReferenceFrame soleFrame;

   private final FramePoint2D desiredCenterOfPressure = new FramePoint2D();
   private final YoDouble copCommandWeight;
   private final Vector2D commandWeight = new Vector2D();
   private final CenterOfPressureCommand centerOfPressureCommand = new CenterOfPressureCommand();

   private final YoDouble startTime;
   private final YoDouble timeExploring;
   private final PartialFootholdControlModule partialFootholdControlModule;

   private int currentCornerIdx = 0;
   private int lastCornerCropped = 0;
   private double lastShrunkTime = 0.0;
   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
   private final FramePoint2D currentCorner = new FramePoint2D();
   private final YoInteger yoCurrentCorner;

   private final FramePoint2D desiredCopInWorld = new FramePoint2D();
   private final YoFramePoint2d yoDesiredCop;

   public ExplorationHelper(ContactableFoot contactableFoot, FootControlHelper footControlHelper, String prefix, YoVariableRegistry registry)
   {
      footholdExplorationActive = new YoBoolean(prefix + "FootholdExplorationActive", registry);
      timeExploring = new YoDouble(prefix + "TimeExploring", registry);
      startTime = new YoDouble(prefix + "StartTime", registry);
      yoCurrentCorner = new YoInteger(prefix + "CurrentCornerExplored", registry);

      centerOfPressureCommand.setContactingRigidBody(contactableFoot.getRigidBody());
      explorationParameters = footControlHelper.getExplorationParameters();
      if (explorationParameters != null)
         copCommandWeight = explorationParameters.getCopCommandWeight();
      else
         copCommandWeight = null;
      soleFrame = footControlHelper.getContactableFoot().getSoleFrame();
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();

      YoGraphicsListRegistry graphicObjectsListRegistry = footControlHelper.getHighLevelHumanoidControllerToolbox().getYoGraphicsListRegistry();
      if (graphicObjectsListRegistry != null)
      {
         yoDesiredCop = new YoFramePoint2d(prefix + "DesiredExplorationCop", ReferenceFrame.getWorldFrame(), registry);
         String name = prefix + "Desired Center of Pressure for Exploration";
         YoArtifactPosition artifact = new YoArtifactPosition(name, yoDesiredCop.getYoX(), yoDesiredCop.getYoY(), GraphicType.BALL, Color.BLUE, 0.003);
         graphicObjectsListRegistry.registerArtifact(prefix + getClass().getSimpleName(), artifact);
      }
      else
      {
         yoDesiredCop = null;
      }
   }

   public void compute(double time, boolean footholdWasUpdated)
   {
      if (!footholdExplorationActive.getBooleanValue() || partialFootholdControlModule == null)
      {
         reset();
         return;
      }

      if (timeExploring.isNaN())
         startTime.set(time);
      timeExploring.set(time - startTime.getDoubleValue());

      computeDesiredCenterOfPressure(time, footholdWasUpdated);

      desiredCopInWorld.setIncludingFrame(desiredCenterOfPressure);
      desiredCopInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      if (yoDesiredCop != null)
         yoDesiredCop.set(desiredCopInWorld);

      centerOfPressureCommand.setDesiredCoP(desiredCenterOfPressure);
      commandWeight.set(copCommandWeight.getDoubleValue(), copCommandWeight.getDoubleValue());
      centerOfPressureCommand.setWeight(commandWeight);
   }

   private void reset()
   {
      yoDesiredCop.setToNaN();
      commandWeight.set(0.0, 0.0);
      centerOfPressureCommand.setWeight(commandWeight);
      lastCornerCropped = 0;
      yoCurrentCorner.set(0);
      timeExploring.set(Double.NaN);
      lastShrunkTime = 0.0;
   }

   private void computeDesiredCenterOfPressure(double time, boolean footholdWasUpdated)
   {
      if (footholdWasUpdated)
      {
         lastCornerCropped = currentCornerIdx + 1;
         lastShrunkTime = time;
      }

      double timeToGoToCorner = explorationParameters.getTimeToGoToCorner().getDoubleValue();
      double timeToStayAtCorner = explorationParameters.getTimeToStayInCorner().getDoubleValue();

      double timeToExploreCorner = timeToGoToCorner + timeToStayAtCorner;
      double timeExploring = time - lastShrunkTime + lastCornerCropped * timeToExploreCorner;

      partialFootholdControlModule.getSupportPolygon(supportPolygon);
      int corners = supportPolygon.getNumberOfVertices();
      currentCornerIdx = (int) (timeExploring / timeToExploreCorner);
      int corner = currentCornerIdx % corners;
      yoCurrentCorner.set(corner);

      supportPolygon.getFrameVertex(corner, currentCorner);
      FramePoint2D centroid = supportPolygon.getCentroid();

      currentCorner.changeFrame(soleFrame);
      centroid.changeFrame(soleFrame);
      desiredCenterOfPressure.setToZero(soleFrame);

      double timeExploringCurrentCorner = timeExploring - currentCornerIdx * timeToExploreCorner;
      if (timeExploringCurrentCorner <= timeToGoToCorner)
      {
         double percent = timeExploringCurrentCorner / timeToGoToCorner;
         percent = MathTools.clamp(percent, 0.0, 1.0);
         desiredCenterOfPressure.interpolate(centroid, currentCorner, percent);
      }
      else
         desiredCenterOfPressure.set(currentCorner);

      if (time - lastShrunkTime > 2.0 * timeToExploreCorner * corners)
      {
         footholdExplorationActive.set(false);
         partialFootholdControlModule.informExplorationDone();
      }
   }

   public boolean isExploring()
   {
      return footholdExplorationActive.getBooleanValue();
   }

   public void startExploring()
   {
      footholdExplorationActive.set(true);
   }

   public void stopExploring()
   {
      footholdExplorationActive.set(false);
   }

   public InverseDynamicsCommand<?> getCommand()
   {
      if (isExploring())
         return centerOfPressureCommand;
      return null;
   }

}
