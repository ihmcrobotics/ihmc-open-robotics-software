package us.ihmc.avatar;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class MeshTerrainObjectViewer
{

   private final YoBoolean showOriginalMeshGraphics;
   private final YoBoolean showDecomposedMeshGraphics;
   private static SimulationConstructionSet scs = null;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private YoVariableServer yoVariableServer = null;
   
   
   
   MeshTerrainObjectViewer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoVariableServer.setMainRegistry(registry, yoGraphicsListRegistry);
      
      String relativeFilePath = "models/Stool/Stool.obj";
      
      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, -Math.PI / 2.0));
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/SmallWalkway/Walkway.obj", configuration);
      
      
      //Initializing variables
      showOriginalMeshGraphics = new YoBoolean("showOriginalMeshGraphics", registry);
      showDecomposedMeshGraphics = new YoBoolean("showDecomposedMeshGraphics", registry);
      this.yoVariableServer = null;
      scs = new SimulationConstructionSet(new Robot("MeshTerrainObjectViewer"));
      scs.startOnAThread();
      


      double groundKz = 500;
      double groundBz = 300.0;
      double groundKxy = 50000.0;
      double groundBxy = 2000.0;
   }
   
   public static void main(String[] args)
   {

      new MeshTerrainObjectViewer();
   }
}


