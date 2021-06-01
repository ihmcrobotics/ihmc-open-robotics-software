package us.ihmc.valkyrie.stepReachability;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

import java.util.Map;

public class StepReachabilityVisualizer
{
   public StepReachabilityVisualizer(Map<FramePose3D, Boolean> reachabilityMap)
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
         validStep.translate(0.0, 0.0, footPose.getYaw()/10);

         // Reachability for this footpose indicated by green/red color
         if (reachabilityMap.get(footPose)) validStep.addSphere(0.015, YoAppearance.Green());
         else validStep.addSphere(0.015, YoAppearance.Red());

         scs.addStaticLinkGraphics(validStep);
      }
   }
}
