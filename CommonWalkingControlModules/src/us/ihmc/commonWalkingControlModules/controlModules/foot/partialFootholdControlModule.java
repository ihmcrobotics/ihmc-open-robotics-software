package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.awt.Color;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.utilities.math.geometry.ConvexPolygonTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;

public class partialFootholdControlModule
{
   private final String name = getClass().getSimpleName();
   public enum PartialFootholdState { FULL, UNSAFE_CORNER, PARTIAL, SAFE_PARTIAL};
   
   private final YoVariableRegistry registry;

   private final EnumYoVariable<PartialFootholdState> footholdState;

   private final FootRotationCalculator footRotationCalculator;

   private final ReferenceFrame soleFrame;

   private final FrameConvexPolygon2d defaultFootPolygon;
   private final FrameConvexPolygon2d shrunkFootPolygon;
   private final FrameConvexPolygon2d unsafePolygon;
   private final YoFrameConvexPolygon2d yoUnsafePolygon;

   private final IntegerYoVariable shrinkMaxLimit;
   private final IntegerYoVariable shrinkCounter;

   private final FrameLine2d lineOfRotation;
   
   public partialFootholdControlModule(String namePrefix, double dt, ContactablePlaneBody contactableFoot, TwistCalculator twistCalculator, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      soleFrame = contactableFoot.getSoleFrame();
      defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());
      shrunkFootPolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      unsafePolygon = new FrameConvexPolygon2d(defaultFootPolygon);
      lineOfRotation = new FrameLine2d(soleFrame);
      
      registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);
      
      footholdState = new EnumYoVariable<>(namePrefix + "PartialFootHoldState", registry, PartialFootholdState.class, true);
      yoUnsafePolygon = new YoFrameConvexPolygon2d(namePrefix + "UnsafeFootPolygon", "", ReferenceFrame.getWorldFrame(), 10, registry);

      shrinkMaxLimit = new IntegerYoVariable(namePrefix + "MaximumNumberOfFootShrink", registry);
      shrinkMaxLimit.set(1);
      shrinkCounter = new IntegerYoVariable(namePrefix + "FootShrinkCounter", registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon yoGraphicPolygon = new YoArtifactPolygon(namePrefix + "UnsafeRegion", yoUnsafePolygon, Color.RED, false);
         yoGraphicsListRegistry.registerArtifact("Partial Foothold", yoGraphicPolygon);
      }

      footRotationCalculator = new FootRotationCalculator(namePrefix, dt, contactableFoot, twistCalculator, yoGraphicsListRegistry, registry);
   }

   public void compute(FramePoint2d desiredCenterOfPressure, FramePoint2d centerOfPressure)
   {
      shrunkFootPolygon.setIncludingFrameAndUpdate(defaultFootPolygon);
      unsafePolygon.setIncludingFrameAndUpdate(defaultFootPolygon);

      footRotationCalculator.compute(centerOfPressure);
      
      if (footRotationCalculator.isFootRotating())
      {
         footRotationCalculator.getLineOfRotation(lineOfRotation);
         
         int numberOfVerticesRemoved = ConvexPolygonTools.cutPolygonWithLine(lineOfRotation, unsafePolygon, RobotSide.LEFT);
         
         if (numberOfVerticesRemoved <= 0)
         {
            doNothing();
         }
         else if (numberOfVerticesRemoved == 1)
         {
            footholdState.set(PartialFootholdState.UNSAFE_CORNER);
            shrinkFoothold(desiredCenterOfPressure);
         }
         else
         {
            footholdState.set(PartialFootholdState.PARTIAL);
            shrinkFoothold(desiredCenterOfPressure);
         }
      }
      else
      {
         doNothing();
      }
   }

   private void doNothing()
   {
      footholdState.set(PartialFootholdState.FULL);
      yoUnsafePolygon.hide();
   }

   private void shrinkFoothold(FramePoint2d desiredCenterOfPressure)
   {
      if (unsafePolygon.isPointInside(desiredCenterOfPressure, 0.0e-3))
      {
         ConvexPolygonTools.cutPolygonWithLine(lineOfRotation, shrunkFootPolygon, RobotSide.RIGHT);
         unsafePolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         yoUnsafePolygon.setFrameConvexPolygon2d(unsafePolygon);
      }
      else
      {
         doNothing();
      }
   }

   private final FramePoint tempPosition = new FramePoint();
   
   public boolean applyShrunkPolygon(YoPlaneContactState contactStateToModify)
   {
      if (footholdState.getEnumValue() != PartialFootholdState.PARTIAL)
         return false;

      if (shrinkCounter.getIntegerValue() >= shrinkMaxLimit.getIntegerValue())
         return false;

      List<YoContactPoint> contactPoints = contactStateToModify.getContactPoints();

      if (contactPoints.size() != shrunkFootPolygon.getNumberOfVertices())
         return false;

      for (int i = 0; i < shrunkFootPolygon.getNumberOfVertices(); i++)
      {
         FramePoint2d vertex = shrunkFootPolygon.getFrameVertex(i);
         tempPosition.setIncludingFrame(vertex.getReferenceFrame(), vertex.getX(), vertex.getY(), 0.0);
         contactPoints.get(i).setPosition(tempPosition);
      }
      
      shrinkCounter.increment();
      return true;
   }
   
   public void reset()
   {
      shrinkCounter.set(0);
      footholdState.set(null);
      yoUnsafePolygon.hide();
      footRotationCalculator.reset();
   }
}
