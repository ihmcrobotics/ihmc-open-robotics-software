package us.ihmc.simulationconstructionset;

import java.awt.Dimension;
import java.awt.Point;
import java.io.File;
import java.util.ArrayList;

import us.ihmc.simulationconstructionset.commands.AllCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.GUIConfigurationSaveAndLoad;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;

public class StandardAllCommandsExecutor implements AllCommandsExecutor
{
   private StandardSimulationGUI standardSimulationGUI;
   private SimulationConstructionSet simulationConstructionSet;
   private DataBuffer dataBuffer;

   private ArrayList<ViewportSelectorCommandListener> viewportSelectorCommandListenersToRegister = new ArrayList<ViewportSelectorCommandListener>();
   private ArrayList<ToggleKeyPointModeCommandListener> toggleKeyPointModeCommandListenersToRegister = new ArrayList<ToggleKeyPointModeCommandListener>();

   public StandardAllCommandsExecutor()
   {
   }

   public void setup(SimulationConstructionSet simulationConstructionSet, StandardSimulationGUI standardSimulationGUI, DataBuffer dataBuffer)
   {
      if (this.simulationConstructionSet != null)
         throw new RuntimeException("this.simulationConstructionSet != null");
      if (this.standardSimulationGUI != null)
         throw new RuntimeException("this.standardSimulationGUI != null");
      if (this.dataBuffer != null)
         throw new RuntimeException("this.dataBuffer != null");

      this.simulationConstructionSet = simulationConstructionSet;
      this.standardSimulationGUI = standardSimulationGUI;
      this.dataBuffer = dataBuffer;

      for (ViewportSelectorCommandListener listener : viewportSelectorCommandListenersToRegister)
      {
         standardSimulationGUI.registerViewportSelectorCommandListener(listener);
      }

      for (ToggleKeyPointModeCommandListener listener : toggleKeyPointModeCommandListenersToRegister)
      {
         dataBuffer.registerToggleKeyPointModeCommandListener(listener);
      }
   }

   public void addCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.addCameraKey();
   }

   public ArrayList<Integer> getCameraKeyPoints()
   {
      return standardSimulationGUI.getCameraKeyPoints();
   }

   public void addKeyPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setKeyPoint();
   }

   /**
    * Gets the KeyPoints in the cropped data
    * 
    * @return The current KeyPoints as an ArrayList of Integer
    */
   public ArrayList<Integer> getKeyPoints()
   {
      return standardSimulationGUI.getKeyPoints();
   }

   public GraphArrayWindow createNewGraphWindow()
   {
      return createNewGraphWindow(null);
   }

   public GraphArrayWindow createNewGraphWindow(String graphGroupName)
   {
      return createNewGraphWindow(graphGroupName, 0, null, null, false);
   }

   public GraphArrayWindow createNewGraphWindow(String graphGroupName, int screenID, Point windowLocation, Dimension windowSize, boolean maximizeWindow)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.createNewGraphWindow(graphGroupName, screenID, windowLocation, windowSize, maximizeWindow);

      return null;
   }

   public GraphArrayWindow getGraphArrayWindow(String windowName)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.getGraphArrayWindow(windowName);

      return null;
   }

   public ViewportWindow createNewViewportWindow()
   {
      return createNewViewportWindow(null);
   }

   public ViewportWindow createNewViewportWindow(String viewportName)
   {
      return createNewViewportWindow(viewportName, 0, false);
   }

   public ViewportWindow createNewViewportWindow(String viewportName, int screenID, boolean maximizeWindow)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.createNewViewportWindow(viewportName, screenID, maximizeWindow);
      else
         return null;
   }

   public ViewportWindow getViewportWindow(String windowName)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.getViewportWindow(windowName);

      return null;
   }

   public void cropBuffer()
   {
      dataBuffer.cropData();
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomFullView();
   }
   
   public void packBuffer()
   {
      dataBuffer.packData();
      if (standardSimulationGUI != null)
         standardSimulationGUI.updateGraphs();
   }
   
   public void cutBuffer()
   {
      dataBuffer.cutData();
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomFullView();
   }
   
   public void thinBuffer(int keepEveryNthPoint)
   {
      dataBuffer.thinData(keepEveryNthPoint);
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomFullView();
   }

   public void gotoInPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.gotoInPoint();
   }

   public void gotoOutPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.gotoOutPoint();
   }

   public void nextCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.nextCameraKey();
   }

   public void play()
   {
      simulationConstructionSet.play();
   }

   public void previousCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.previousCameraKey();
   }

   public void removeCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.removeCameraKey();
   }

   public void setInPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setInPoint();
      else
         dataBuffer.setInPoint();
   }

   public void setOutPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setOutPoint();
      else
         dataBuffer.setOutPoint();
   }
   
   public void setInOutPointFullBuffer()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setInOutPointFullBuffer();
      else
         dataBuffer.setInOutPointFullBuffer();
   }

   public void simulate()
   {
      simulationConstructionSet.simulate();
   }

   public boolean isSimulating()
   {
      return simulationConstructionSet.isSimulating();
   }

   public void stepBackward()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.stepBackward();
      else
      {
         dataBuffer.tick(-1);
      }
   }

   public void stepForward()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.stepForward();
      else
      {
         dataBuffer.tick(1);
      }
   }

   public void stop()
   {
      simulationConstructionSet.stop();
   }

   public void toggleCameraKeyMode()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.toggleCameraKeyMode();
   }

   public boolean isKeyPointModeToggled()
   {
      return dataBuffer.isKeyPointModeToggled();
   }

   public void toggleKeyPointMode()
   {
      dataBuffer.toggleKeyPointMode();
   }

   public void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener)
   {
      if (dataBuffer != null)
         dataBuffer.registerToggleKeyPointModeCommandListener(commandListener);
      else
      {
         toggleKeyPointModeCommandListenersToRegister.add(commandListener);
      }
   }

   public void selectViewport(String viewportName)
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.selectViewport(viewportName);
   }

   public void hideViewport()
   {
      if (standardSimulationGUI != null)
         EventDispatchThreadHelper.invokeLater(new Runnable()
         {
            public void run()
            {
               standardSimulationGUI.hideViewport();
            }
         });
   }

   public void showViewport()
   {
      if (standardSimulationGUI != null)
         EventDispatchThreadHelper.invokeLater(new Runnable()
         {
            public void run()
            {
               standardSimulationGUI.showViewport();
            }
         });
   }

   public boolean isViewportHidden()
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.isViewportHidden();
      return true;
   }

   public void registerViewportSelectorCommandListener(ViewportSelectorCommandListener commandListener)
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.registerViewportSelectorCommandListener(commandListener);
      else
      {
         viewportSelectorCommandListenersToRegister.add(commandListener);
      }
   }

   public void zoomIn()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomIn();
   }

   public void zoomOut()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomOut();
   }

   public void selectGUIConfigFromFile(String fullPath)
   {
      GUIConfigurationSaveAndLoad defaultSaveAndLoad = new GUIConfigurationSaveAndLoad(simulationConstructionSet, standardSimulationGUI);
      defaultSaveAndLoad.loadGUIConfiguration(fullPath);
   }

   public void setPlaybackRealTimeRate(double realtimeRate)
   {
      simulationConstructionSet.setPlaybackRealTimeRate(realtimeRate);
   }

   public double getPlaybackRealTimeRate()
   {
      return simulationConstructionSet.getPlaybackRealTimeRate();
   }

   public int getInPoint()
   {
      return simulationConstructionSet.getInPoint();
   }

   public void setIndex(int index)
   {
      simulationConstructionSet.setIndex(index);
   }

   public void setIndexButDoNotNotifySimulationRewoundListeners(int index)
   {
      simulationConstructionSet.setIndexButDoNotNotifySimulationRewoundListeners(index);
   }

   public boolean tick(int ticks)
   {
      return simulationConstructionSet.tick(ticks);
   }

   public boolean tickButDoNotNotifySimulationRewoundListeners(int ticks)
   {
      return simulationConstructionSet.tickButDoNotNotifySimulationRewoundListeners(ticks);
   }

   public int getIndex()
   {
      return simulationConstructionSet.getIndex();
   }

   public boolean isIndexBetweenInAndOutPoint(int indexToCheck)
   {
      return simulationConstructionSet.isIndexBetweenInAndOutPoint(indexToCheck);
   }

   public int getOutPoint()
   {
      return simulationConstructionSet.getOutPoint();
   }

   public void exportSnapshot(File snapshotFile)
   {
      simulationConstructionSet.exportSnapshot(snapshotFile);
   }

   public void enableGUIComponents()
   {
      simulationConstructionSet.enableGUIComponents();
   }

   public void disableGUIComponents()
   {
      simulationConstructionSet.disableGUIComponents();
   }

   private boolean alreadyStartedClosing = false;
   
   public void closeAndDispose()
   {
      if (alreadyStartedClosing) return;
      
      alreadyStartedClosing = true;
      
      this.standardSimulationGUI = null;
      this.simulationConstructionSet = null;
      this.dataBuffer = null;

      if (viewportSelectorCommandListenersToRegister != null)
      {
         for (ViewportSelectorCommandListener viewportSelectorCommandListener : viewportSelectorCommandListenersToRegister)
         {
            viewportSelectorCommandListener.closeAndDispose();
         }
         viewportSelectorCommandListenersToRegister.clear();
      }

      if (toggleKeyPointModeCommandListenersToRegister != null)
      {
         for (ToggleKeyPointModeCommandListener toggleKeyPointModeCommandListener : toggleKeyPointModeCommandListenersToRegister)
         {
            toggleKeyPointModeCommandListener.closeAndDispose();
         }
         toggleKeyPointModeCommandListenersToRegister.clear();
      }

      viewportSelectorCommandListenersToRegister = null;
      toggleKeyPointModeCommandListenersToRegister = null;
   }

}
