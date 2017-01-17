package us.ihmc.simulationconstructionset.util.graphics;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiSliderBoard;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class GraphicsExample
{
   public GraphicsExample()
   {
      // create registries...
      YoGraphicsListRegistry registries = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("GraphicsExample");
      YoGraphicsList yoGraphicsList = new YoGraphicsList("GraphicsExampleGraphicList");

      // create scs
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("NullBot"));
      scs.setGroundVisible(false);


      // create geometry
      YoFramePoint testPoint = new YoFramePoint("testPoint", "", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint testPoint2 = new YoFramePoint("testPoint2", "", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoint worldOrigin = new YoFramePoint("worldOrigin", "", ReferenceFrame.getWorldFrame(), registry);
      DoubleYoVariable yaw = new DoubleYoVariable("yaw", registry);
      DoubleYoVariable pitch = new DoubleYoVariable("pitch", registry);
      DoubleYoVariable roll = new DoubleYoVariable("roll", registry);

      YoGraphicCoordinateSystem worldCoordinateSystem = new YoGraphicCoordinateSystem("Example", worldOrigin.getYoX(), worldOrigin.getYoY(),
                                                                worldOrigin.getYoZ(), yaw, pitch, roll, 1.0);

      // setup slider board
      setupEvolution(scs, testPoint);

      DoubleYoVariable zero = new DoubleYoVariable("zero", registry);
      zero.set(0.0);

      // create graphics vector
      YoGraphicVector dynamicGraphicVector = new YoGraphicVector("Example", zero, zero, zero, testPoint.getYoX(), testPoint.getYoY(), testPoint.getYoZ(), 1.0,
                                                     YoAppearance.Black());

      // add graphic objects to list
      yoGraphicsList.add(dynamicGraphicVector);

      yoGraphicsList.add(new YoGraphicPosition("Example", testPoint, 0.01, YoAppearance.Red()));
      yoGraphicsList.add(new YoGraphicPosition("Example", testPoint2, 0.01, YoAppearance.Black()));
      yoGraphicsList.add(worldCoordinateSystem);

      // register lists with registries
      registries.registerYoGraphicsList(yoGraphicsList);

      // tell the scs about the registries
      scs.addYoGraphicsListRegistry(registries);

      // start scs
      Thread thread = new Thread(scs);
      thread.start();

      testPoint.set(0.5, 0.5, 0.5);
      testPoint2.set(0.6, 0.6, 0.6);
   }

   private void setupEvolution(SimulationConstructionSet scs, YoFramePoint pointToControl)
   {
      MidiSliderBoard evolution = new MidiSliderBoard(scs);

      double min = 0.0;
      double max = 1.0;

      evolution.setSlider(1, pointToControl.getYoX(), min, max, 1.0);
      evolution.setSlider(2, pointToControl.getYoY(), min, max, 1.0);
      evolution.setSlider(3, pointToControl.getYoZ(), min, max, 1.0);
   }

   public static void main(String[] args)
   {
      @SuppressWarnings("unused")
      GraphicsExample example = new GraphicsExample();
   }

}
