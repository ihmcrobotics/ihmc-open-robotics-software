package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class CenterOfMassStabilityMarginRegionCalculatorVisualizer
{
   private static final boolean showSupportRegion = true;

   public CenterOfMassStabilityMarginRegionCalculatorVisualizer(WholeBodyContactStateInterface input)
   {
      runWithSCS2(input);
   }

   private void runWithSCS2(WholeBodyContactStateInterface input)
   {
      SimulationConstructionSet2 scs2 = new SimulationConstructionSet2(PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      StabilityMarginRegionCalculator calculator = StabilityMarginRegionCalculator.createForCoMStabilityMargin("", 1.0, scs2.getRootRegistry(), graphicsListRegistry);
      calculator.setupForStabilityMarginCalculation(FramePoint3D::new);
      calculator.updateContactState(input);

      List<YoGraphicDefinition> graphicDefinitions = YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry);
      scs2.addYoGraphics(graphicDefinitions);

      YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), scs2.getRootRegistry());
      scs2.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicPoint3D("optimizedCoMViz", optimizedCoM, 0.03, ColorDefinitions.Red()));

      YoFramePoint3D[] contactPoints = new YoFramePoint3D[input.getNumberOfContactPoints()];
      YoFrameVector3D[] contactForces = new YoFrameVector3D[input.getNumberOfContactPoints()];
      for (int i = 0; i < input.getNumberOfContactPoints(); i++)
      {
         contactPoints[i] = new YoFramePoint3D("contactPoint" + i, ReferenceFrame.getWorldFrame(), scs2.getRootRegistry());
         contactForces[i] = new YoFrameVector3D("contactForce" + i, ReferenceFrame.getWorldFrame(), scs2.getRootRegistry());
         scs2.addYoGraphic(YoGraphicDefinitionFactory.newYoGraphicArrow3D("contactForceViz" + i, contactPoints[i], contactForces[i], 0.2, ColorDefinitions.Red()));
         contactPoints[i].setFromReferenceFrame(input.getContactFrame(i));
      }

      YoDouble[] yoDualSolution = new YoDouble[50];
      for (int i = 0; i < yoDualSolution.length; i++)
      {
         yoDualSolution[i] = new YoDouble("dual" + i, scs2.getRootRegistry());
      }

      LogTools.info("----- Support Region -----");
      scs2.start(true, true, false);

      for (int i = 0; i < 18; i++)
      {
         calculator.performUpdateForNextVertex(false);

         Point2DReadOnly optimizedStabilityPoint = calculator.getOptimizationModule().getOptimizedStabilityPoint();
         optimizedCoM.set(optimizedStabilityPoint);

         for (int j = 0; j < input.getNumberOfContactPoints(); j++)
         {
            calculator.getOptimizationModule().getResolvedForce(j, contactForces[j]);
         }

         DMatrixRMaj solverDualSolution = calculator.getSolverDualSolution(i);
         for (int j = 0; j < Math.min(solverDualSolution.getNumRows(), yoDualSolution.length); j++)
         {
            yoDualSolution[j].set(solverDualSolution.get(j));
         }

         scs2.simulateNow(1);
      }

      ConvexPolygon2DReadOnly supportRegion0 = calculator.getFeasibleRegion();
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

         ConvexPolygon2DReadOnly supportRegion = calculator.getFeasibleRegion();
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

      new CenterOfMassStabilityMarginRegionCalculatorVisualizer(input);
   }
}
