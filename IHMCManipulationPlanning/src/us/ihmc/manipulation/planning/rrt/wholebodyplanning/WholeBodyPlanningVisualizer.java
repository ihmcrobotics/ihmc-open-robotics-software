package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import java.awt.Color;

import javax.swing.JSplitPane;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.thread.ThreadTools;

public class WholeBodyPlanningVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoGraphicsListRegistry yoGraphicsListRegistry2 = new YoGraphicsListRegistry();
   private SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
   private SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("InhoRobot"), parameters);
   
   private final double trajectoryTime = 10.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 100);
   
   private final YoFramePoint yoPointInho = new YoFramePoint("inho", worldFrame, registry);

   
   
   
   
   

   
   public WholeBodyPlanningVisualizer()
   {        
      YoGraphicPosition inhoYoFramePoint = new YoGraphicPosition("inhoYoFramePoint", yoPointInho, 0.025, YoAppearance.AliceBlue());      
      yoGraphicsListRegistry.registerYoGraphic("inho", inhoYoFramePoint);
      
      YoFramePoint firstCircleCenter = new YoFramePoint("firstCircleCenter", ReferenceFrame.getWorldFrame(), registry);   
      DoubleYoVariable firstCircleRadius = new DoubleYoVariable("firstRadius", registry);
      YoArtifactOval firstCircle = new YoArtifactOval("inscribedCircle", firstCircleCenter, firstCircleRadius, Color.GREEN);
      firstCircleRadius.set(0.05);      
      firstCircleCenter.setX(0.01);
      firstCircleCenter.setY(0.01);
      yoGraphicsListRegistry.registerArtifact("first", firstCircle);      
      

      YoFramePoint secondCircleCenter = new YoFramePoint("secondCircleCenter", ReferenceFrame.getWorldFrame(), registry);   
      DoubleYoVariable secondCircleRadius = new DoubleYoVariable("secondRadius", registry);
      YoArtifactOval secondCircle = new YoArtifactOval("inscribedCircle", secondCircleCenter, secondCircleRadius, Color.BLACK);      
      secondCircleRadius.set(0.1);      
      secondCircleCenter.setX(0.0);
      secondCircleCenter.setY(0.1);
      yoGraphicsListRegistry2.registerArtifact("secondCircle", secondCircle);
      
      
      YoFramePoint thirdCircleCenter = new YoFramePoint("thirdCircleCenter", ReferenceFrame.getWorldFrame(), registry);   
      DoubleYoVariable thirdCircleRadius = new DoubleYoVariable("thirdRadius", registry);
      YoArtifactOval thirdCircle = new YoArtifactOval("inscribedCircleaaaa", thirdCircleCenter, thirdCircleRadius, Color.RED);      
      thirdCircleRadius.set(0.1);      
      thirdCircleCenter.setX(0.1);
      thirdCircleCenter.setY(0.0);
      yoGraphicsListRegistry2.registerArtifact("thirdCircle", thirdCircle);
      
      
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      
      
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setVariableNameToTrack("centroidGraphic");
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.setCreateInSeperateWindow(true);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory2 = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory2.setPlotterName("Plotter2");
      simulationOverheadPlotterFactory2.setVariableNameToTrack("centroidGraphic");
      simulationOverheadPlotterFactory2.setShowOnStart(true);
      simulationOverheadPlotterFactory2.addYoGraphicsListRegistries(yoGraphicsListRegistry2);
      simulationOverheadPlotterFactory2.setCreateInSeperateWindow(true);
      simulationOverheadPlotterFactory2.createOverheadPlotter();
      
      
      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         FramePoint currentPosition = new FramePoint(worldFrame, 0.2, -0.2 + 0.4/trajectoryTime*t, 0.5);
         yoPointInho.set(currentPosition);
         
         

         
         
         scs.tickAndUpdate();
      }
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }


   public static void main(String[] args)
   {
      new WholeBodyPlanningVisualizer();
   }
}
