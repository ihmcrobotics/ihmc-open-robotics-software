package us.ihmc.avatar;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;

import org.apache.commons.io.FilenameUtils;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import javafx.application.Platform;
import javafx.scene.control.Button;
import javafx.stage.FileChooser;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObjectParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This class can be used to view convex decomposition results
 * <p>
 * Convex decomposition parameters can be tuned via YoVariables and once updateVisuals boolean is
 * checked, the updated results will be shown Note: Not all the Convex Decompsoition parameters are
 * available to tune currently. I have only included the one that would make the most impact. They
 * can be added later
 * </p>
 * 
 * @author KhizarMohammed AmjedMohamed
 */
public class MeshTerrainObjectViewer
{
   private final YoInteger maximumNumberOfHulls;
   private final YoInteger maximumNumberOfVerticesPerHull;
   private final YoInteger maximumVoxelResolution;
   private final YoDouble maximumVolumetricPercentError;

   private final YoBoolean showRawMeshGraphics;
   private final YoBoolean showDecomposedMeshGraphics;
   private final YoBoolean doConvexDecomposition;

   private final YoBoolean updateVisuals;

   private MeshTerrainObjectParameters parameters;
   private MeshTerrainObject meshTerrainObject;

   private static SimulationConstructionSet2 scs = null;
   private final YoRegistry registry = new YoRegistry("DecomopsitionParameters");

   private List<VisualDefinition> visuals = null;

   private File currentMeshFile = null;

   public MeshTerrainObjectViewer()
   {

      MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters();

      maximumNumberOfHulls = new YoInteger("maximumNumberOfHulls", registry);
      maximumNumberOfHulls.setVariableBounds(0, 0);

      maximumNumberOfVerticesPerHull = new YoInteger("maximumNumberOfVerticesPerHull", registry);
      maximumNumberOfVerticesPerHull.setVariableBounds(0, 0);

      maximumVoxelResolution = new YoInteger("maximumVoxelResolution", registry);
      maximumNumberOfVerticesPerHull.setVariableBounds(100000, 10000000);

      maximumVolumetricPercentError = new YoDouble("maximumVolumetricPercentError", registry);
      maximumNumberOfVerticesPerHull.setVariableBounds(0, 0);

      showRawMeshGraphics = new YoBoolean("showRawMeshGraphics", registry);
      showDecomposedMeshGraphics = new YoBoolean("showDecomposedMeshGraphics", registry);
      doConvexDecomposition = new YoBoolean("doConvexDecomposition", registry);

      updateVisuals = new YoBoolean("UpdateVisuals", registry);
      updateVisuals.set(false);

      maximumNumberOfHulls.set(parameters.getMaxNoOfHulls());
      maximumNumberOfVerticesPerHull.set(parameters.getMaxNoOfVertices());
      maximumVoxelResolution.set(parameters.getVoxelResolution());

      maximumVolumetricPercentError.set(parameters.getMaxVolumePercentError());

      showRawMeshGraphics.set(parameters.isShowUndecomposedMeshGraphics());
      showDecomposedMeshGraphics.set(parameters.isShowDecomposedMeshGraphics());
      doConvexDecomposition.set(parameters.isDoConvexDecomposition());

      scs = new SimulationConstructionSet2("MeshTerrainObjectViewer");
      scs.addRegistry(registry);

      this.parameters = new MeshTerrainObjectParameters();

      scs.startSimulationThread();
      scs.waitUntilVisualizerFullyUp();

      Platform.runLater(() ->
      {
         // Loading a new terrain mesh into the GUI
         Button loadTerrainMeshButton = new Button("Load Terrain Mesh");
         loadTerrainMeshButton.setOnAction(e -> loadMeshTerrain());
         scs.addCustomGUIControl(loadTerrainMeshButton);

         //Saving terrain Mesh Parameters in the GUI
         Button saveTerrainMeshParams = new Button("Save terrain mesh params");
         saveTerrainMeshParams.setOnAction(e -> saveTerrainMeshParameters());
         scs.addCustomGUIControl(saveTerrainMeshParams);
      });
   }

   private void loadMeshTerrain()
   {
      FileChooser fileChooser = new FileChooser();
      File defaultFilePath = SessionVisualizerIOTools.getDefaultFilePath("MeshTerrain");
      if (defaultFilePath != null)
         fileChooser.setInitialDirectory(defaultFilePath);
      File result = fileChooser.showOpenDialog(scs.getPrimaryGUIWindow());
      if (result == null)
         return;
      SessionVisualizerIOTools.setDefaultFilePath("MeshTerrain", result);
      makeMeshTerrainObject(result.getAbsolutePath());
      this.parameters = meshTerrainObject.getParameters();
      updateYoVariables();
      updateGraphics();
      currentMeshFile = result;
   }

   private void saveTerrainMeshParameters()
   {
      if (currentMeshFile == null)
      {
         LogTools.error("Couldn't save parameters");
         return;
      }

      File parentFile = currentMeshFile.getParentFile();
      String meshName = FilenameUtils.removeExtension(currentMeshFile.getName());
      File parameterFile = new File(parentFile, meshName + MeshTerrainObject.VHACD_FILENAME_EXTENSION);
      try
      {
         if (parameterFile.exists())
            parameterFile.delete();
         parameterFile.createNewFile();
         JsonFactory jsonFactory = new JsonFactory();
         JsonGenerator generator = jsonFactory.createGenerator(new FileOutputStream(parameterFile));
         generator.useDefaultPrettyPrinter();
         generator.writeStartObject();

         generator.writeNumberField(maximumNumberOfHulls.getName(), maximumNumberOfHulls.getValue());
         generator.writeNumberField(maximumNumberOfVerticesPerHull.getName(), maximumNumberOfVerticesPerHull.getValue());
         generator.writeNumberField(maximumVolumetricPercentError.getName(), maximumVolumetricPercentError.getValue());
         generator.writeNumberField(maximumVoxelResolution.getName(), maximumVoxelResolution.getValue());

         generator.writeBooleanField(showDecomposedMeshGraphics.getName(), showDecomposedMeshGraphics.getValue());
         generator.writeBooleanField(showRawMeshGraphics.getName(), showRawMeshGraphics.getValue());
         generator.writeBooleanField(doConvexDecomposition.getName(), doConvexDecomposition.getValue());

         generator.writeEndObject();
         generator.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void makeMeshTerrainObject(String pathToMesh)
   {
      if (pathToMesh == null)
         return;
      meshTerrainObject = new MeshTerrainObject(pathToMesh);
   }

   private void makeMeshTerrainObject(MeshTerrainObjectParameters parameters)
   {
      if (parameters == null || parameters.getModelDirectory() == null)
         return;

      meshTerrainObject = new MeshTerrainObject(parameters);
   }

   private void updateGraphics()
   {
      if (visuals != null)
         scs.removeStaticVisuals(visuals);
      visuals = VisualsConversionTools.toVisualDefinitions(meshTerrainObject.getLinkGraphics());
      scs.addStaticVisuals(visuals);
   }

   private MeshTerrainObjectParameters getParameters()
   {
      return this.parameters;
   }

   private void updateParameters()
   {
      this.parameters.setMaxNoOfHulls(maximumNumberOfHulls.getValue());
      this.parameters.setMaxNoOfVertices(maximumNumberOfVerticesPerHull.getValue());
      this.parameters.setVoxelResolution(maximumVoxelResolution.getValue());

      this.parameters.setMaxVolumePercentError(maximumVolumetricPercentError.getValue());

      this.parameters.setShowDecomposedMeshGraphics(showDecomposedMeshGraphics.getValue());
      this.parameters.setShowUndecomposedMeshGraphics(showRawMeshGraphics.getValue());
      this.parameters.setDoConvexDecomposition(doConvexDecomposition.getValue());
   }

   private void updateYoVariables()
   {
      this.maximumNumberOfHulls.set(this.parameters.getMaxNoOfHulls());
      this.maximumNumberOfVerticesPerHull.set(this.parameters.getMaxNoOfVertices());
      this.maximumVoxelResolution.set(this.parameters.getVoxelResolution());

      this.maximumVolumetricPercentError.set(this.parameters.getMaxVolumePercentError());

      this.showDecomposedMeshGraphics.set(this.parameters.isShowDecomposedMeshGraphics());
      this.showRawMeshGraphics.set(this.parameters.isShowUndecomposedMeshGraphics());
      this.doConvexDecomposition.set(this.parameters.isDoConvexDecomposition());
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
            viewer.makeMeshTerrainObject(viewer.getParameters());
            viewer.updateGraphics();
         }
      });

      scs.initializeBufferSize(1000);
      scs.setRealTimeRateSimulation(true);
      scs.start(true, false, false);
      scs.simulate();
   }

}
