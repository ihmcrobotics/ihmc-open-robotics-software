package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.GroundContactPoint;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

public class DRCSimulationVisualizer
{
   public DRCSimulationVisualizer(SDFRobot sdfRobot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("Simulation Viz");

      ArrayList<GroundContactPoint> groundContactPoints = sdfRobot.getAllGroundContactPoints();
      AppearanceDefinition appearance = YoAppearance.Red(); // BlackMetalMaterial();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         double scaleFactor = 0.0015;
         DynamicGraphicVector dynamicGraphicVector = new DynamicGraphicVector(groundContactPoint.getName(), groundContactPoint, scaleFactor, appearance);
         dynamicGraphicObjectsList.add(dynamicGraphicVector);
      }
      
      if (dynamicGraphicObjectsListRegistry != null)
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
   }
}
