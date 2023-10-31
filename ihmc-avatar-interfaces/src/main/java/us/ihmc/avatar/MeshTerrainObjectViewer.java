package us.ihmc.avatar;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerranObjectParameters;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerranObjectParameters.ConvexDecomposition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
/**
 * This class can be used to view convex decomposition results
 *<p> Convex decomposition parameters can be tuned via YoVariables and once updateVisuals boolean is checked, the updated results will be shown
 * Note: Not all the Convex Decompsoition parameters are available to tune currently. I have only included the one that would make the most impact.
 * They can be added later
 *</p>
 * @author Khizar Mohammed Amjed Mohamed
 */
public class MeshTerrainObjectViewer
{
   private final YoInteger maximumNumberOfHulls;
   private final YoInteger maximumNumberOfVerticesPerHull;
   private final YoInteger maximumVoxelResolution;

   private final YoDouble maximumVolumetricPercentError;
   private final YoDouble maximumConcavity;

   private final YoBoolean showOriginalMeshGraphics;
   private final YoBoolean showDecomposedMeshGraphics;
   private final YoEnum<ConvexDecomposition> decompositionType; 
   
   private final YoBoolean updateVisuals;

   private MeshTerranObjectParameters parameters;
   private MeshTerrainObject meshTerrainObject;
   
   private static SimulationConstructionSet2 scs = null;
   private final YoRegistry registry = new YoRegistry("DecomopsitionParameters");

   private List<VisualDefinition> visuals = null;

   //private String relativeFilePath = "models/stool/stool.obj";
   private String relativeFilePath = "models/valley/valley.obj";

   public MeshTerrainObjectViewer()
   {

      MeshTerranObjectParameters parameters = new MeshTerranObjectParameters();

      maximumNumberOfHulls = new YoInteger("maxNoOfHulls", registry);
      maximumNumberOfVerticesPerHull = new YoInteger("maxNoOfVertices", registry);
      maximumVoxelResolution = new YoInteger("maxVoxelResolution", registry);

      maximumVolumetricPercentError = new YoDouble("MaxVolumePercentError", registry);
      maximumConcavity = new YoDouble("MaxConcavity", registry);

      showOriginalMeshGraphics = new YoBoolean("ShowOriginalMeshGraphics", registry);
      showDecomposedMeshGraphics = new YoBoolean("ShowDecomposedMeshGraphics", registry);
      decompositionType = new YoEnum<>("DecompositionType", registry, ConvexDecomposition.class);
      
      updateVisuals = new YoBoolean("UpdateVisuals", registry);

      maximumNumberOfHulls.set(parameters.getMaxNoOfHulls());
      maximumNumberOfVerticesPerHull.set(parameters.getMaxNoOfVertices());
      maximumVoxelResolution.set(parameters.getVoxelResolution());

      maximumVolumetricPercentError.set(parameters.getMaxVolumePercentError());
      maximumConcavity.set(parameters.getMaxConvacity());

      showOriginalMeshGraphics.set(parameters.isShowUndecomposedMeshGraphics());
      showDecomposedMeshGraphics.set(parameters.isShowDecomposedMeshGraphics());
      decompositionType.set(parameters.getDecompositionType());
      
      scs = new SimulationConstructionSet2("MeshTerrainObjectViewer");
      scs.addRegistry(registry);

      
      this.parameters = new MeshTerranObjectParameters();

      updateParameters();
      makeMeshTerrainObject();
      updateGraphics();

      scs.startSimulationThread();
   }

   private void makeMeshTerrainObject()
   {
      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, -Math.PI / 2.0));
      meshTerrainObject = new MeshTerrainObject(relativeFilePath, parameters);
   }


   private void updateGraphics()
   {
      if (visuals != null)
         scs.removeStaticVisuals(visuals);
      visuals = VisualsConversionTools.toVisualDefinitions(meshTerrainObject.getLinkGraphics());
      scs.addStaticVisuals(visuals);
   }

   private void updateParameters()
   {
      // TODO Auto-generated method stub
      this.parameters.setMaxNoOfHulls(maximumNumberOfHulls.getValue());
      this.parameters.setMaxNoOfVertices(maximumNumberOfVerticesPerHull.getValue());
      this.parameters.setVoxelResolution(maximumVoxelResolution.getValue());

      this.parameters.setMaxVolumePercentError(maximumVolumetricPercentError.getValue());
      this.parameters.setMaxConvacity(maximumConcavity.getValue());

      this.parameters.setShowDecomposedMeshGraphics(showDecomposedMeshGraphics.getValue());
      this.parameters.setShowUndecomposedMeshGraphics(showOriginalMeshGraphics.getValue());
      
      this.parameters.setDecompositionType(decompositionType.getValue());

   }

   public static void main(String[] args)
   {
      MeshTerrainObjectViewer viewer = new MeshTerrainObjectViewer();
      scs.addAfterPhysicsCallback(time ->
      {
         if (viewer.updateVisuals.getValue())
         {
            viewer.updateVisuals.set(false);
            viewer.updateParameters();
            viewer.makeMeshTerrainObject();
            viewer.updateGraphics();
         }
      });

      scs.initializeBufferSize(1000);
      scs.setRealTimeRateSimulation(true);
      scs.start(true, false, false);
      scs.simulate();
   }

}
