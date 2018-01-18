package us.ihmc.robotics.parameterGui.gui;

import java.util.List;

import javafx.beans.Observable;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.robotics.parameterGui.tree.ParameterTree;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class GuiController
{
   @FXML
   private TextField searchField;

   @FXML
   private CheckBox hideNamespaces;

   @FXML
   private ParameterTree tree;

   private List<Registry> registries;

   public void initialize()
   {
      searchField.textProperty().addListener(observable -> handleSearch(observable));
   }

   @FXML
   protected void handleNamespaceButton(ActionEvent event)
   {
      tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
   }

   protected void handleSearch(Observable observable)
   {
      tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
   }

   public void setRegistries(List<Registry> registries)
   {
      this.registries = registries;
      tree.setRegistries(registries, hideNamespaces.isSelected(), searchField.getText());
   }
}
