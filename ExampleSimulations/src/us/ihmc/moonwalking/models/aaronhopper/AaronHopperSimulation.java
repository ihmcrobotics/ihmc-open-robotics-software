package us.ihmc.moonwalking.models.aaronhopper;

import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class AaronHopperSimulation
{
   public AaronHopperSimulation()
   {
      AaronHopperRobot Hopper = new AaronHopperRobot("AaronHopperRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(Hopper);

      CameraConfiguration camera1 = new CameraConfiguration("camera1");
      camera1.setCameraPosition(0, -27.239, 2.4159);
      camera1.setCameraFix(0, 0, 0.6);
      scs.setupCamera(camera1);
      scs.selectCamera("camera1");

      LinearGroundContactModel Ground = new LinearGroundContactModel(Hopper, new YoVariableRegistry("AaronHopperRegistry"));
      Hopper.setGroundContactModel(Ground);

      Thread simThread = new Thread(scs);
      simThread.start();
   }

   public static void main(String[] args)
   {
      new AaronHopperSimulation();
   }

}
