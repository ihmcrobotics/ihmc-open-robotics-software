package us.ihmc.avatar.multiContact.pushRecovery;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public interface ReactiveBracingPlanner
{
   /**
    * Generates plan for optimal hand-wall contact placement, given the robot's current state and upcoming footsteps
    */
   void plan(FrameVector2DReadOnly desiredToCurrentCapturePoint,
                    FramePoint3DReadOnly centerOfMassPosition,
                    FrameConvexPolygon2DReadOnly supportPolygon,
                    SideDependentList<? extends FramePoint3DReadOnly> handPositions,
                    SideDependentList<MutableBoolean> areFeetInContact,
                    ReferenceFrame midFeetZUpFrame,
                    SideDependentList<HandContactCommand> contactCommandsToPack,
                    double stanceWidth);
   /**
    * Sets the detected list of planar regions
    */
   void setPlanarRegions(PlanarRegionsListCommand planarRegionsListCommand);

   /**
    * YoRegistry for graphics, etc.
    */
   YoRegistry getRegistry();

   default void triggerDiagnosticInference()
   {
      // do nothing
   }

   /**
    * SCS 2 YoGraphics
    */
   YoGraphicDefinition getSCS2YoGraphics();
}
