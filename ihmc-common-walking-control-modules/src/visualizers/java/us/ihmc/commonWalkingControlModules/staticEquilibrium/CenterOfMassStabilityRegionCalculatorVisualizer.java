package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

import java.util.List;

public class CenterOfMassStabilityRegionCalculatorVisualizer
{
   private static final boolean showSupportRegion = true;

   public CenterOfMassStabilityRegionCalculatorVisualizer(WholeBodyContactStateInterface input)
   {
      runWithSCS2(input);
   }

   private void runWithSCS2(WholeBodyContactStateInterface input)
   {
      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2(PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());
      scs2.start(true, true, false);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      CenterOfMassStaticStabilityRegionCalculator calculator = new CenterOfMassStaticStabilityRegionCalculator("", 1.0, scs2.getRootRegistry(), graphicsListRegistry);
      calculator.updateContactState(input);

      List<YoGraphicDefinition> graphicDefinitions = YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry);
      scs2.addYoGraphics(graphicDefinitions);

      LogTools.info("----- Support Region -----");

      while (!calculator.hasSolvedWholeRegion())
      {
         calculator.performCoMRegionQuery();
         scs2.simulateNow(1);
      }

      ConvexPolygon2DReadOnly supportRegion0 = calculator.getFeasibleCoMRegion();
      for (int i = 0; i < supportRegion0.getNumberOfVertices(); i++)
      {
         System.out.println("\t" + supportRegion0.getVertex(i));
      }

      VisualDefinitionFactory supportRegionGraphics = new VisualDefinitionFactory();

      for (int i = 0; i < input.getNumberOfContactPoints(); i++)
      {
         FramePoint3D contactPoint = new FramePoint3D(input.getContactFrame(i));
         FrameVector3D surfaceNormal = new FrameVector3D(input.getContactFrame(i), Axis3D.Z);

         contactPoint.changeFrame(ReferenceFrame.getWorldFrame());
         surfaceNormal.changeFrame(ReferenceFrame.getWorldFrame());

         supportRegionGraphics.identity();
         supportRegionGraphics.appendTranslation(contactPoint);
         supportRegionGraphics.addSphere(0.03, new ColorDefinition());

         Quaternion surfaceOrientation = new Quaternion();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, surfaceOrientation);
         supportRegionGraphics.appendRotation(surfaceOrientation);
         supportRegionGraphics.appendTranslation(0.0, 0.0, -0.01);

         ColorDefinition beige = new ColorDefinition();
         beige.setRed(255);
         beige.setGreen(255);
         beige.setBlue(180);
         supportRegionGraphics.addCylinder(0.02, 0.15, beige);
      }

      if (showSupportRegion)
      {
         double renderedHeight = 0.0;

         ConvexPolygon2DReadOnly supportRegion = calculator.getFeasibleCoMRegion();
         supportRegionGraphics.identity();
         supportRegionGraphics.appendTranslation(0.0, 0.0, renderedHeight);

         ColorDefinition glass = new ColorDefinition();
         glass.setRed(120);
         glass.setGreen(180);
         glass.setBlue(240);
         glass.setAlpha(0.5);
         supportRegionGraphics.addExtrudedPolygon(supportRegion, 0.01, glass);
      }

      supportRegionGraphics.identity();

      scs2.addStaticVisuals(supportRegionGraphics.getVisualDefinitions());
      scs2.cropBuffer();
      scs2.startSimulationThread();

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
//      WholeBodyContactStateInterface input = ContactStateExamples.createTriangleFlatGround();
//      WholeBodyContactStateInterface input = ContactStateExamples.createTriangleTiltedOutSlightly();
//      WholeBodyContactStateInterface input = ContactStateExamples.createTriangleTiltedOutALot();
//      WholeBodyContactStateInterface input = ContactStateExamples.createTriangleOneTiltedFullyOut();
//      WholeBodyContactStateInterface input = ContactStateExamples.createTriangleOneTiltedFullyIn();
//      WholeBodyContactStateInterface input = ContactStateExamples.createFlatSquare();
//      WholeBodyContactStateInterface input = ContactStateExamples.createBipedFeet();
//      WholeBodyContactStateInterface input = ContactStateExamples.createBipedFeetWithSingleHandhold();
      WholeBodyContactStateInterface input = ContactStateExamples.createBipedFeetWithTwoHandholds();

      new CenterOfMassStabilityRegionCalculatorVisualizer(input);
   }
}
