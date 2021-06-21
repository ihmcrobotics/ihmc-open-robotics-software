package us.ihmc.avatar.reachabilityMap.footstep;

import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

import java.awt.*;
import java.util.Map;

public class StepReachabilityVisualizer
{
   public StepReachabilityVisualizer(StepReachabilityData stepReachabilityData)
   {
      // Set up SCS and coordinate object
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 16000);
      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);
      Graphics3DObject coordinate = new Graphics3DObject();
      coordinate.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(coordinate);
      scs.setGroundVisible(false);
      scs.setCameraFix(0.0, 0.0, 1.0);
      scs.setCameraPosition(8.0, 0.0, 3.0);
      scs.startOnAThread();
      double yawDivisions = stepReachabilityData.getYawDivisions();

      for (StepReachabilityLatticePoint latticePoint : stepReachabilityData.getLegReachabilityMap().keySet())
      {
         // Represent foot position as sphere, yaw as z-axis translation
         Graphics3DObject validStep = new Graphics3DObject();
         validStep.translate(latticePoint.getXIndex(), latticePoint.getYIndex(), 0.0);
         validStep.rotate(Math.toRadians(90), new Vector3D(0,1,0));
         validStep.rotate(Math.toRadians(latticePoint.getYawIndex() * 10), new Vector3D(1,0,0));

         // Reachability for this foot position indicated by green/red color
         double reachabilityValue = stepReachabilityData.getLegReachabilityMap().get(latticePoint);
         if (reachabilityValue > 40) reachabilityValue = 40;
         AppearanceDefinition appearance = YoAppearance.RGBColor(reachabilityValue/40, (40-reachabilityValue)/40, 0);
         validStep.addArrow(0.3, appearance, appearance);

         scs.addStaticLinkGraphics(validStep);
      }
   }
}
