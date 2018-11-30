package us.ihmc.parameterTuner.guiElements.main;

import java.util.List;

import javafx.scene.Node;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;

public interface ParameterGuiInterface
{
   /**
    * This will be called periodically by the GUI to check if the registry structure needs reloading.
    * If that is the case the GUI will call {@link #getRegistriesCopy()} and update all parameters.
    */
   public boolean pollReloadAll();

   /**
    * This will be called by the GUI to load the registry structure if a reload is requested through
    * {@link #pollReloadAll()}. The returned registry will be modified by the GUI.
    */
   public List<GuiRegistry> getRegistriesCopy();

   /**
    * This will be called periodically by the GUI if the user modifies any parameter. The list will
    * contain copies of all parameters that were changed by the user. The GUI will not hold on to a
    * reference of the changed parameters so they are safe to modify.
    */
   public void submitChangedParameters(List<GuiParameter> changedParameters);

   /**
    * This will be called periodically by the GUI to get a list with parameters that were updated by
    * other entities such as a different tuning GUI or a YoVariableServer. Note, that the GUI will
    * only update the value of the parameters. Description and suggested bounds will not be updated.
    */
   public List<GuiParameter> pollUpdatedParameters();

   /**
    * This can be used to provide the GUI with a interface for controlling the input manager. If the
    * value is null no node will be added. For example, in the case of a tuning GUI implementation
    * that uses files, this node could contain 'open' and 'save' buttons that will be added to the main
    * window.
    */
   public Node getInputManagerNode();

   /**
    * This will be called once by the GUI to inform the input manager that the program was shut down.
    * This will allow the input manager to close any YoVariableServers and clean up.
    */
   public void shutdown();

   /**
    * Allows the GUI to modify the root registry. This is used when the user would like to save an XML
    * file that uses a non-root registry as a new root. For example if the registry structure seen in
    * the GUI is "scs" - "atlas" - "controller" but the user of the GUI would like to save an XML file
    * for the controller.
    */
   public void changeRootRegistries(List<String> pollRootRegistryNames);
}
