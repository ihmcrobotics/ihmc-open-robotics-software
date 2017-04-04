package us.ihmc.simulationConstructionSetTools.socketCommunication;

import us.ihmc.simulationconstructionset.NewDataListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.hierarchyTree.NameSpaceHierarchyTree;

import java.util.ArrayList;

public class RobotSocketConnectionCreator
{
   public static RobotSocketConnection createTCPConnectionToHost(String host, SimulationConstructionSet scs)
   {
      return createTCPConnectionToHost(host, (ArrayList<NewDataListener>)  null, scs);
   }

   public static RobotSocketConnection createTCPConnectionToHost(String host, NewDataListener newDataListener, SimulationConstructionSet scs)
   {
      if (newDataListener == null)
      {
         return createTCPConnectionToHost(host, scs);
      }

      ArrayList<NewDataListener> newDataListeners = new ArrayList<NewDataListener>(1);
      newDataListeners.add(newDataListener);

      return createTCPConnectionToHost(host, newDataListeners, scs);
   }

   /**
    * Creates a SCSRobotGUICommandListener which communicates to a robot at the specified ethernet HOST address.
    * Only the variables specified in the YoVariableRegistries, set through the GUI are sent and/or logged on the robot side.
    * This method adds the relevant GUI modifications for robot communication.
    * Once communication is established the new RobotSocketConnection monitors and manages robot communication.
    *
    * @param host IP address of robot.
    * @return The new SCSRobotGUICommandListener.
    * @see SCSRobotGUICommunicatorComponents SCSRobotGUICommandListener
    */
   public static RobotSocketConnection createTCPConnectionToHost(String host, ArrayList<NewDataListener> newDataListeners, SimulationConstructionSet scs)
   {
      RobotConnectionGUIUpdater guiUpdater = new RobotConnectionGUIUpdater(scs);

      GUISideCommandListener robotCommandListener = new GUISideCommandListener(scs.getDataBuffer(), scs.getRootRegistry(), guiUpdater, guiUpdater);
      RobotSocketConnection robotSocketConnection = new RobotSocketConnection(host, robotCommandListener, scs.getRootRegistry(), newDataListeners);

      if (scs.getGUI() != null)
      {
         NameSpaceHierarchyTree nameSpaceHierarchyTree = scs.getGUI().getCombinedVarPanel().getNameSpaceHierarchyTree();
         nameSpaceHierarchyTree.addRegistrySettingsChangedListener(robotSocketConnection);
         robotCommandListener.addCreatedNewRegistryListener(nameSpaceHierarchyTree);
      }

      SCSRobotGUICommunicatorComponents robotGUI = new SCSRobotGUICommunicatorComponents(robotSocketConnection);

      robotCommandListener.attachDoDisconnectListener(robotSocketConnection);
      robotCommandListener.attachDoDisconnectListener(robotGUI);

      robotGUI.putButtonsAndExitActionListenerOnSimulationGUI(scs);

      return robotSocketConnection;
   }
}
