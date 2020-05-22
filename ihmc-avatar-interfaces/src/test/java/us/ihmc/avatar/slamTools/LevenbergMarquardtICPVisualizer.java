package us.ihmc.avatar.slamTools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

public class LevenbergMarquardtICPVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 50.0;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);

   private final YoFramePoint3D modelLocation;
   private final YoFrameQuaternion modelOrientation;
   private final YoFramePoint3D dataLocation;
   private final YoFrameQuaternion dataOrientation;

   private final List<YoDouble[]> yoPointsHolder;
   int numberOfPoints = 100;

   public LevenbergMarquardtICPVisualizer()
   {
      modelLocation = new YoFramePoint3D("model_location_", worldFrame, registry);
      modelOrientation = new YoFrameQuaternion("model_orientation_", worldFrame, registry);
      dataLocation = new YoFramePoint3D("data_location_", worldFrame, registry);
      dataOrientation = new YoFrameQuaternion("data_orientation_", worldFrame, registry);

      YoGraphicCoordinateSystem modelFrame = new YoGraphicCoordinateSystem("model_", modelLocation, modelOrientation, 0.5, YoAppearance.Red());
      YoGraphicCoordinateSystem dataFrame = new YoGraphicCoordinateSystem("data_", dataLocation, dataOrientation, 0.5, YoAppearance.Blue());

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("model_graphics", modelFrame);
      yoGraphicsListRegistry.registerYoGraphic("data_graphics", dataFrame);

      YoGraphicPosition[] pointCloudViz = new YoGraphicPosition[numberOfPoints];

      yoPointsHolder = new ArrayList<>();
      YoGraphicsList yoGraphicPointCloudListRegistry = new YoGraphicsList("model_pointcloudviz");
      for (int i = 0; i < numberOfPoints; i++)
      {
         YoDouble[] pointArray = new YoDouble[3];
         for (int j = 0; j < 3; j++)
         {
            pointArray[j] = new YoDouble("Point_" + i + "_" + j, registry);
         }
         yoPointsHolder.add(pointArray);
         pointCloudViz[i] = new YoGraphicPosition(i
               + "pointviz", yoPointsHolder.get(i)[0], yoPointsHolder.get(i)[1], yoPointsHolder.get(i)[2], 0.02, YoAppearance.Blue());

         yoGraphicPointCloudListRegistry.add(pointCloudViz[i]);
      }
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicPointCloudListRegistry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);

      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         pointCloudViz[(int) t].setPosition(0.0, 0.0, t / 100);

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new LevenbergMarquardtICPVisualizer();
   }
}
