package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

/**
 *
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: Class for storing camera view configurations for display in a ViewportPanel.<br />
 * <br />
 * This class can store the position and size information of several cameras as shown in the following example:<br />
 * <br />
 * {@code view2.addCameraView("camera 1", 0, 0, 1, 2);}<br />
 *  {@code view2.addCameraView("camera 2", 1, 0, 3, 2);}<br />
 *  {@code view2.addCameraView("robot cam", 0, 2, 2, 1);}<br />
 * <br />
 * The above code adds three cameras named camera 1, camera 2 and robot cam. The next component of each camera is its column and row.
 * In this example camera's 1 and 2 are both in the same row, row 0, but are separated into columns 0 and 1 respectively.
 * The final two components, width and height, indicate the number of columns and rows that the camera fills.  Camera 1 has a width of 1 and a height of 2.
 * When placing cameras ensure that no overlap occurs as it will cause unpredictable behavior.
 * </p>
 *
 * @author Jerry Pratt
 * @version 1.0
 */
public class ViewportConfiguration
{
   private String name;
   private ArrayList<ViewportPanelConfiguration> panelConfigurations = new ArrayList<ViewportPanelConfiguration>();

//   private ArrayList<Object> cachedStandard3DViews = null;
//   private ArrayList<Object> cachedPanels = null;

   /**
    * Creates an empty configuration with the specified name.
    *
    * @param name Name of the configuration.
    */
   public ViewportConfiguration(String name)
   {
      this.name = name;
   }

   /**
    * Accessor method for the ViewportConfiguration's name
    *
    * @return Name of the configuration
    */
   public String getName()
   {
      return name;
   }

   /**
    * Adds a new camera view to this configuration.  The new camera is placed in the specified row and column and is the specified number of rows and columns in size.
    *
    * @param cameraName Name of the camera to use.
    * @param col Column in which to place the camera.
    * @param row Row in which to place the camera.
    * @param width Width of the camera in number of columns.
    * @param height Height of the camera in number of rows.
    */
   public void addCameraView(String cameraName, int col, int row, int width, int height)
   {
      ViewportPanelConfiguration panelConfig = new ViewportPanelConfiguration(cameraName, col, row, width, height);
      panelConfigurations.add(panelConfig);
   }

   /**
    * Retrieves a list of the stored ViewportPanelConfigurations.  Each of these contain the relevant information to display a camera view.
    *
    * @return List containing the ViewportPanelConfigurations stored in this ViewportConfiguration.
    */
   public ArrayList<ViewportPanelConfiguration> getPanelConfigurations()
   {
      return panelConfigurations;
   }

}
