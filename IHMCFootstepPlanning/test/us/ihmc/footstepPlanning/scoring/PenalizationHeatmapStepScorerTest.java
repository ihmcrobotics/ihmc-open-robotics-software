package us.ihmc.footstepPlanning.scoring;

import java.awt.Color;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphics3DDescription.appearance.AppearanceDefinition;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.color.Gradient;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.tools.thread.ThreadTools;

public class PenalizationHeatmapStepScorerTest
{
   private Color[] scoreColorGradient;
   private Point3d footstepPlannerGoal = new Point3d(4.0, 1.0, 0.3);
   private ConvexPolygon2d defaultFootPolygon;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoVariableRegistry registry;
   private Map<Integer, YoGraphicPolygon> candidateFootstepPolygons;
   private DoubleYoVariable colorIndexYoVariable;
   private IntegerYoVariable candidateIndex;
   
   private double incrementX = 0.1;
   private double incrementY = 0.05;
   private double yMin = -1.0;
   private double xMax = 1.0;
   private double xMin = -0.8;
   private int numInX = (int) ((xMax - xMin) / incrementX);
   private int numInY = (int) ((xMax - yMin) / incrementY);
   private Random random = new Random(89324823L);

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 3000000)
   public void testScoringFootsteps()
   {
      SimulationTestingParameters simulationTestingParameters = new SimulationTestingParameters();
      simulationTestingParameters.setCreateGUI(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      Robot robot = new Robot("Test");
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      registry = scs.getRootRegistry();

      DoubleYoVariable scoreYoVariable = new DoubleYoVariable("score", registry);
      DoubleYoVariable xYoVariable = new DoubleYoVariable("x", registry);
      DoubleYoVariable yYoVariable = new DoubleYoVariable("y", registry);
      colorIndexYoVariable = new DoubleYoVariable("colorIndex", registry);
      candidateIndex = new IntegerYoVariable("candidateIndex", registry);
      
      PenalizationHeatmapStepScorer footstepScorer = new PenalizationHeatmapStepScorer(registry, yoGraphicsListRegistry, null);

      candidateFootstepPolygons = new HashMap<>();

      scoreColorGradient = Gradient.createGradient(Color.GREEN, Color.RED, 1000);

      defaultFootPolygon = PlanningTestTools.createDefaultFootPolygon();
      FramePose idealFramePose = new FramePose(ReferenceFrame.getWorldFrame());
      idealFramePose.setPosition(0.4, 0.0, 0.31);
      createFootstep("ideal", -1, YoAppearance.HotPink());
      candidateFootstepPolygons.get(-1).setPose(idealFramePose);
      FramePose swingStartFramePose = new FramePose(ReferenceFrame.getWorldFrame());
      swingStartFramePose.setPosition(0.0, 0.0, 0.31);
      createFootstep("swingStart", -2, YoAppearance.Blue());
      candidateFootstepPolygons.get(-2).setPose(swingStartFramePose);

      int candidateFootstepIndex = 0;
      for (double y = yMin; y < xMax; y += incrementY)
      {
         for (double x = xMin; x < xMax; x += incrementX)
         {
            createFootstep("candidate", candidateFootstepIndex++, YoAppearance.Red());
         }
      }

      robot.setController(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return new YoVariableRegistry("blah");
         }

         @Override
         public String getName()
         {
            return "blah";
         }

         @Override
         public String getDescription()
         {
            return null;
         }

         @Override
         public void doControl()
         {
            DoubleYoVariable yoTime = (DoubleYoVariable) registry.getVariable("t");

            candidateIndex.set((int) yoTime.getDoubleValue() % (numInX * numInY));
            
            double x = xMin + (((int) candidateIndex.getIntegerValue()) % numInX) * incrementX;
            double y = yMin + (((int) candidateIndex.getIntegerValue()) / numInX) * incrementY;

            FramePose candidateFramePose = new FramePose(ReferenceFrame.getWorldFrame());
            candidateFramePose.setPosition(x, y, 0.3 + (y - 0.2) * 0.2);

            double score = footstepScorer.scoreFootstep(null, swingStartFramePose, idealFramePose, candidateFramePose, footstepPlannerGoal);
            scoreYoVariable.set(score);
            xYoVariable.set(x);
            yYoVariable.set(y);

            candidateFootstepPolygons.get(candidateIndex.getIntegerValue()).setPose(candidateFramePose);
            candidateFootstepPolygons.get(candidateIndex.getIntegerValue()).updateAppearance(getAppearanceForScore(scoreYoVariable.getDoubleValue()));
            
            scs.getStandardSimulationGUI().updateSimulationGraphics();
         }
      });

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-1.0, 0.0, 30.0);
      scs.setScrollGraphsEnabled(false);
      scs.startOnAThread();

      scs.setDT(1, 1);
      scs.simulate(numInX * numInY);

      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }
   }

   private void createFootstep(String name, int index, AppearanceDefinition appearance)
   {
      YoFramePose footstepYoFramePose = new YoFramePose(name + index + "FramePose", ReferenceFrame.getWorldFrame(), registry);
      footstepYoFramePose.setPosition(0.0, 0.0, -1.0);
      YoGraphicPolygon footstepYoGraphicPolygon = new YoGraphicPolygon(name + index + "YoGraphicPolygon", footstepYoFramePose,
                                                                       defaultFootPolygon.getNumberOfVertices(), registry, 1.0, appearance);
      footstepYoGraphicPolygon.updateConvexPolygon2d(defaultFootPolygon);
      candidateFootstepPolygons.put(index, footstepYoGraphicPolygon);
      yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), footstepYoGraphicPolygon);
   }

   private AppearanceDefinition getAppearanceForScore(double score)
   {
      double greenScore = 0.0;
      double redScore = -1.5;

      score -= greenScore;
      score = -score;

      double index = score * 1000.0 / (greenScore - redScore);
      index = MathTools.clipToMinMax(index, 0, 999);

      colorIndexYoVariable.set(index);

      Color scoreColor = scoreColorGradient[(int) (index)];
      return new YoAppearanceRGBColor(scoreColor, 0.0);
   }
}
