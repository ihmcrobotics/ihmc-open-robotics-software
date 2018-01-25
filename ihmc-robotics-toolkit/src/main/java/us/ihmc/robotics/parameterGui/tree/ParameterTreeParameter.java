package us.ihmc.robotics.parameterGui.tree;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.layout.HBox;
import javafx.scene.text.Text;
import us.ihmc.robotics.parameterGui.GuiParameter;

public class ParameterTreeParameter implements ParameterTreeValue
{
   private final GuiParameter parameter;
   private ParameterNode node;

   public ParameterTreeParameter(GuiParameter parameter)
   {
      this.parameter = parameter;
   }

   @Override
   public boolean isRegistry()
   {
      return false;
   }

   @Override
   public String getName()
   {
      return parameter.getName();
   }

   public GuiParameter getParameter()
   {
      return parameter;
   }

   @Override
   public Node getOrCreateNode()
   {
      if (node == null)
      {
         node = new ParameterNode(parameter);
      }
      return node;
   }

   private class ParameterNode extends HBox
   {
      private static final String FXML_PATH = "parameter_node.fxml";

      @FXML
      private Text name;
      @FXML
      private Label value;

      public ParameterNode(GuiParameter parameter)
      {
         FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
         loader.setRoot(this);
         loader.setController(this);
         try
         {
            loader.load();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         parameter.addChangedListener(p -> value.setText(parameter.getCurrentValue()));
         name.setText(parameter.getName());
         value.setText(parameter.getCurrentValue());
      }
   }
}
