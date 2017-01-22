package us.ihmc.javaFXToolkit.graphing;

import javax.vecmath.AxisAngle4d;

import javafx.embed.swing.JFXPanel;
import javafx.scene.SceneAntialiasing;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.scenes.View3DFactory.SceneType;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.text.Text3D;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;

public class JavaFX3DGraph
{
   private final JFXPanel javaFXPanel;
   private final GraphIndicesHolder graphIndicesHolder;
   private final SelectedVariableHolder selectedVariableHolder;
   private final DataEntryHolder dataEntryHolder;
   private final TimeDataHolder timeDataHolder;

   public JavaFX3DGraph(GraphIndicesHolder graphIndicesHolder, SelectedVariableHolder selectedVariableHolder, DataEntryHolder dataEntryHolder,
                        TimeDataHolder timeDataHolder)
   {
      javaFXPanel = new JFXPanel();
      this.graphIndicesHolder = graphIndicesHolder;
      this.selectedVariableHolder = selectedVariableHolder;
      this.dataEntryHolder = dataEntryHolder;
      this.timeDataHolder = timeDataHolder;

      View3DFactory view3dFactory = new View3DFactory(-1, -1, true, SceneAntialiasing.BALANCED, SceneType.MAIN_SCENE);
      view3dFactory.addCameraController(0.0, 1e7, false);
      view3dFactory.setBackgroundColor(Color.LIGHTGRAY);
      
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(7));
      double infinity = 10000.0;
      double lineWidth = 0.005;
      double start = lineWidth / 2.0;
      meshBuilder.addLine(-start, 0.0, 0.0, -infinity, 0.0, 0.0, lineWidth, Color.GRAY);
      meshBuilder.addLine(0.0, -start, 0.0, 0.0, -infinity, 0.0, lineWidth, Color.GRAY);
      meshBuilder.addLine(0.0, 0.0, -start, 0.0, 0.0, -infinity, lineWidth, Color.GRAY);
      meshBuilder.addLine(start, 0.0, 0.0, infinity, 0.0, 0.0, lineWidth, Color.hsb(Color.RED.getHue(), 1.0, 1.0));
      meshBuilder.addLine(0.0, start, 0.0, 0.0, infinity, 0.0, lineWidth, Color.hsb(Color.GREEN.getHue(), 1.0, 1.0));
      meshBuilder.addLine(0.0, 0.0, start, 0.0, 0.0, infinity, lineWidth, Color.hsb(Color.BLUE.getHue(), 1.0, 1.0));
      MeshView coordinateSystem = new MeshView(meshBuilder.generateMesh());
      coordinateSystem.setMaterial(meshBuilder.generateMaterial());
      view3dFactory.addNodeToView(coordinateSystem);
      
      double fontHeight = 0.1;
      double fontThickness = lineWidth;
      
      Text3D xLabel = new Text3D("X");
      xLabel.setFontThickness(fontThickness);
      xLabel.setFontHeight(fontHeight);
      xLabel.setFontColor(Color.BLACK);
      xLabel.setOrientation(new AxisAngle4d(1.0, -1.0, 0.0, 180.0));
      xLabel.setPosition(new TransformablePoint3d(1.0, -fontHeight / 2.0, 0.0));
      view3dFactory.addNodeToView(xLabel.getNode());
      
      Text3D yLabel = new Text3D("Y");
      yLabel.setFontThickness(fontThickness);
      yLabel.setFontHeight(fontHeight);
      yLabel.setFontColor(Color.BLACK);
      yLabel.setOrientation(new AxisAngle4d(1.0, 0.0, 0.0, 180.0));
      yLabel.setPosition(new TransformablePoint3d(fontHeight / 2.0, 1.0, 0.0));
      view3dFactory.addNodeToView(yLabel.getNode());
      
      Text3D zLabel = new Text3D("Z");
      zLabel.setFontThickness(fontThickness);
      zLabel.setFontHeight(fontHeight);
      zLabel.setFontColor(Color.BLACK);
      zLabel.setOrientation(new AxisAngle4d(1.0, 0.0, 0.0, -90.0));
      zLabel.setPosition(new TransformablePoint3d(fontHeight / 2.0, 0.0, 1.0));
      view3dFactory.addNodeToView(zLabel.getNode());

      javaFXPanel.setScene(view3dFactory.getScene());
   }

   public JFXPanel getPanel()
   {
      return javaFXPanel;
   }
}
