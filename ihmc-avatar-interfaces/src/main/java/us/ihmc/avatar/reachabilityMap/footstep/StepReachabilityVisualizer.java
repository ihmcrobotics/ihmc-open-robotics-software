package us.ihmc.avatar.reachabilityMap.footstep;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

import java.awt.*;
import java.util.Map;

public class StepReachabilityVisualizer
{
   public StepReachabilityVisualizer(Map<FramePose3D, Double> reachabilityMap, int queriesPerAxis)
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

      for (FramePose3D footPose : reachabilityMap.keySet())
      {
         // Represent footpose as sphere, yaw as z-axis translation
         Graphics3DObject validStep = new Graphics3DObject();
         validStep.translate(footPose.getPosition());
         validStep.translate(0.0, 0.0, footPose.getYaw()/queriesPerAxis);

         // Reachability for this footpose indicated by green/red color
         double reachabilityValue = reachabilityMap.get(footPose);
         if (reachabilityValue > 40) reachabilityValue = 40;
         System.out.println("reachabilityValue: " + reachabilityValue);
         AppearanceDefinition appearance = YoAppearance.RGBColor(reachabilityValue/40, (40-reachabilityValue)/40, 0);
         validStep.addSphere(0.1/queriesPerAxis, appearance);

         scs.addStaticLinkGraphics(validStep);
      }
   }
}
