package us.ihmc.simulationconstructionset;

public class SimulationConstructionSetParameters
{
   private boolean showSplashScreen = true;
   private boolean createGUI = true;
   private boolean showWindows = true;
   private int dataBufferSize = 8192;
   
   public SimulationConstructionSetParameters()
   {
      
   }

   public SimulationConstructionSetParameters(boolean createGUI, int bufferSize)
   {
      this.createGUI = createGUI;
      this.dataBufferSize = bufferSize;
   }

   public SimulationConstructionSetParameters(int dataBufferSize)
   {
      this.dataBufferSize = dataBufferSize;
   }

   public SimulationConstructionSetParameters(boolean createGUI)
   {
      this.createGUI = createGUI;
   }
   
   public static SimulationConstructionSetParameters createFromEnvironmentVariables()
   {
      SimulationConstructionSetParameters parametersToReturn = new SimulationConstructionSetParameters();

      parametersToReturn.setFromSystemProperties();

      return parametersToReturn;
   }
   
   public void setFromSystemProperties()
   {
      String property = System.getProperty("create.scs.gui");
      if (property != null)
      {
         Boolean createSCSGUI = Boolean.parseBoolean(property);
         setCreateGUI(createSCSGUI);
      }
      
      property = System.getProperty("show.scs.windows");
      if (property != null)
      {
         Boolean showSCSWindows = Boolean.parseBoolean(property);
         setShowWindows(showSCSWindows);
         setShowSplashScreen(showSCSWindows);
      }
      
      property = System.getProperty("scs.dataBuffer.size");
      if (property != null)
      {
         Integer dataBufferSize = Integer.parseInt(property);
         if (dataBufferSize != null) 
         {
            setDataBufferSize(dataBufferSize);
         }
      }
   }

   public int getInitialDataBufferSize()
   {
      return dataBufferSize;
   }

   public boolean getCreateGUI()
   {
      return createGUI;
   }

   public void setCreateGUI(boolean createGUI)
   {
      this.createGUI = createGUI; 
   }

   public void setDataBufferSize(int dataBufferSize)
   {
      this.dataBufferSize = dataBufferSize;      
   }

   public void setShowSplashScreen(boolean showSplashScreen)
   {
      this.showSplashScreen = showSplashScreen;      
   }

   public boolean getShowWindows()
   {
      return showWindows;
   }

   public void setShowWindows(boolean showWindow)
   {
      this.showWindows = showWindow;
   }

   public boolean getShowSplashScreen()
   {
      return showSplashScreen;
   }
}
