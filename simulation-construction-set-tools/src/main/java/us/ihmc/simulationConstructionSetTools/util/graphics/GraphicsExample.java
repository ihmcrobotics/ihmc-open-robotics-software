package us.ihmc.simulationConstructionSetTools.util.graphics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.MidiSliderBoard;

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
      YoFramePoint3D testPoint = new YoFramePoint3D("testPoint", "", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D testPoint2 = new YoFramePoint3D("testPoint2", "", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoint3D worldOrigin = new YoFramePoint3D("worldOrigin", "", ReferenceFrame.getWorldFrame(), registry);
      YoDouble yaw = new YoDouble("yaw", registry);
      YoDouble pitch = new YoDouble("pitch", registry);
      YoDouble roll = new YoDouble("roll", registry);

      YoGraphicCoordinateSystem worldCoordinateSystem = new YoGraphicCoordinateSystem("Example", worldOrigin.getYoX(), worldOrigin.getYoY(),
                                                                worldOrigin.getYoZ(), yaw, pitch, roll, 2.0);

      // setup slider board
      setupEvolution(scs, testPoint);

      YoDouble zero = new YoDouble("zero", registry);
      zero.set(0.0);

      // create graphics vector
      YoGraphicVector yoGraphicVector = new YoGraphicVector("Example", zero, zero, zero, testPoint.getYoX(), testPoint.getYoY(), testPoint.getYoZ(), 1.0,
                                                     YoAppearance.Black());

      // add graphic objects to list
      yoGraphicsList.add(yoGraphicVector);

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

   private void setupEvolution(SimulationConstructionSet scs, YoFramePoint3D pointToControl)
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
