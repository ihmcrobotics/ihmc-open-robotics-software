package us.ihmc.javaFXToolkit.graphing;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import javafx.embed.swing.JFXPanel;
import javafx.scene.SceneAntialiasing;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.scenes.View3DFactory.SceneType;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoJavaFXHeatmapGraph implements ChangeListener
{

   private final JFXPanel javaFXPanel;
   private final GraphIndicesHolder graphIndicesHolder;
   private final SelectedVariableHolder selectedVariableHolder;
   private final DataEntryHolder dataEntryHolder;
   private final TimeDataHolder timeDataHolder;

   private DoubleYoVariable x;
   private DoubleYoVariable y;

   public YoJavaFXHeatmapGraph(YoVariableRegistry registry, GraphIndicesHolder graphIndicesHolder, SelectedVariableHolder selectedVariableHolder,
                                 DataEntryHolder dataEntryHolder, TimeDataHolder timeDataHolder)
   {
      javaFXPanel = new JFXPanel();
      this.graphIndicesHolder = graphIndicesHolder;
      this.selectedVariableHolder = selectedVariableHolder;
      this.dataEntryHolder = dataEntryHolder;
      this.timeDataHolder = timeDataHolder;

      selectedVariableHolder.setSelectedVariable(registry.getVariable("t"));
      
      selectedVariableHolder.addChangeListener(this);
      
      new View3DFactory(-1, -1, true, SceneAntialiasing.BALANCED, SceneType.MAIN_SCENE);
      
   }

   public void setXVariable(DoubleYoVariable x)
   {
      this.x = x;
   }

   public void setYVariable(DoubleYoVariable y)
   {
      this.y = y;
   }

   public JFXPanel getPanel()
   {
      return javaFXPanel;
   }

   @Override
   public void stateChanged(ChangeEvent changeEvent)
   {
   }
}
