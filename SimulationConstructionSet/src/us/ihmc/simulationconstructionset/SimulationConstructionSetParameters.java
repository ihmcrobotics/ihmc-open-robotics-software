package us.ihmc.simulationconstructionset;

public class SimulationConstructionSetParameters
{
   private boolean showSplashScreen = true;
   private boolean createGUI = true;
   private boolean showWindow = true;
   private int initialDataBufferSize = 8192;
   
   public SimulationConstructionSetParameters()
   {
      
   }

   public SimulationConstructionSetParameters(boolean createGUI, int bufferSize)
   {
      this.createGUI = createGUI;
      this.initialDataBufferSize = bufferSize;
   }

   public SimulationConstructionSetParameters(int bufferSize)
   {
      this.initialDataBufferSize = bufferSize;
   }

   public SimulationConstructionSetParameters(boolean createGUI)
   {
      this.createGUI = createGUI;
   }

   public int getInitialDataBufferSize()
   {
      return initialDataBufferSize;
   }

   public boolean getCreateGUI()
   {
      return createGUI;
   }

   public void setCreateGUI(boolean createGUI)
   {
      this.createGUI = createGUI; 
   }

   public void setInitialDataBufferSize(int initialDataBufferSize)
   {
      this.initialDataBufferSize = initialDataBufferSize;      
   }

   public void setShowSplashScreen(boolean showSplashScreen)
   {
      this.showSplashScreen = showSplashScreen;      
   }

   public boolean getShowWindow()
   {
      return showWindow;
   }

   public void setShowWindow(boolean showWindow)
   {
      this.showWindow = showWindow;
   }

   public boolean getShowSplashScreen()
   {
      return showSplashScreen;
   }
}
