package us.ihmc.simulationconstructionset;

public class SimulationConstructionSetParameters
{
   private boolean showSplashScreen = true;
   private boolean createGUI = true;
   private boolean showWindows = true;
   private int dataBufferSize = 8192;
   private boolean showYoGraphicObjects = true;
   private double yoGraphicsGlobalScale = 1.0;

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
         setDataBufferSize(dataBufferSize);
      }
      
      property = System.getProperty("show.scs.yographics");
      if (property != null)
      {
         Boolean showYoGraphicsObjects = Boolean.parseBoolean(property);
         setShowYoGraphicObjects(showYoGraphicsObjects);
      }

      property = System.getProperty("scs.yographics.globalscale");
      if (property != null)
      {
         Double yoGraphicsGlobalScale = Double.parseDouble(property);
         setYoGraphicsGlobalScale(yoGraphicsGlobalScale);
      }
   }

   public int getDataBufferSize()
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
   
   public boolean getShowYoGraphicObjects()
   {
      return showYoGraphicObjects;
   }

   public void setShowYoGraphicObjects(boolean showYoGraphicObjects)
   {
      this.showYoGraphicObjects = showYoGraphicObjects;
   }

   public double getYoGraphicsGlobalScale()
   {
      return yoGraphicsGlobalScale;
   }
   
   public void setYoGraphicsGlobalScale(double yoGraphicsGlobalScale)
   {
      this.yoGraphicsGlobalScale = yoGraphicsGlobalScale;
   }
 
   @Override
   public String toString()
   {
      String st = "showSplashScreen: " + showSplashScreen + "\n";    
      st += "createGUI: " + createGUI + "\n";    
      st += "showWindows: " + showWindows + "\n";    
      st += "dataBufferSize: " + dataBufferSize + "\n";    
      st += "showYoGraphicObjects: " + showYoGraphicObjects + "\n";    
      st += "yoGraphicsGlobalScale: " + yoGraphicsGlobalScale + "\n";    
      return st;   
   }
}
