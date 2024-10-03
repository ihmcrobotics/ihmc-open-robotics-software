package us.ihmc.avatar.networkProcessor.referenceSpreading;

import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class ReplayHandHybridTrajectoryRS
{
   private TrajectoryRecordReplay trajectoryPlayer;

   private Long startTime;
   private Double startCSVTime;

   private boolean paused = false;
   private boolean finished = false;

   private HashMap<String, Double> thisFrame = new HashMap<>();
   private HashMap<String, Double> nextFrame = new HashMap<>();

   public ReplayHandHybridTrajectoryRS(String filePath, int numberOfParts)
   {
      trajectoryPlayer = new TrajectoryRecordReplay(filePath, numberOfParts, true);
      trajectoryPlayer.importData(true);
      //        System.out.println("KeyMatrix: " + trajectoryPlayer.getKeyMatrix());
   }

   public void start(boolean restart)
   {
      this.startTime = null;
      LogTools.info("Starting replay");
      if (!paused || restart)
      {
         reset();
         makeMap(trajectoryPlayer.play(true), thisFrame);
         makeMap(trajectoryPlayer.play(true), nextFrame);
         startCSVTime = thisFrame.get("root.time[sec]");
         finished = false;
      }
      paused = false;
      trajectoryPlayer.setDoneReplay(false);
      if (startCSVTime == null)
      {
         throw new IllegalStateException("startCSVTime is null");
      }
   }

   public void start(){
      start(true);
   }

   public void pause(){
      LogTools.info("Pausing replay");
      paused = true;
   }

   public void stop()
   {
      LogTools.info("Stopping replay");
      reset();
      finished = true;
   }

   public void doAction(Long currentTime)
   {
      if (paused || finished)
      {
         return;
      }

      if (startTime == null)
      {
         if (!Objects.equals(startCSVTime, thisFrame.get("root.time[sec]")))
         {
            startTime = (long) (currentTime - thisFrame.get("root.time[sec]") * 1_000_000_000);
         }
         else
         {
            startTime = currentTime;
         }
      }
      if (trajectoryPlayer.hasDoneReplay()){
         stop();
      }
      nextFrame(currentTime);
   }

   private void nextFrame(Long currentTime)
   {
      while ((currentTime - startTime) / 1_000_000_000.0 >= nextFrame.get("root.time[sec]") - startCSVTime)
      {
         thisFrame.putAll(nextFrame);
         makeMap(trajectoryPlayer.play(true), nextFrame);
      }
      LogTools.info("NextFrame Time: " + nextFrame.get("root.time[sec]") + " Current Time: " + (currentTime - startTime) / 1_000_000_000.0);
   }

   private void reset()
   {
      trajectoryPlayer.reset();
      thisFrame.clear();
      nextFrame.clear();
   }

   private void makeMap(double[] values, HashMap<String, Double> mapToPack)
   {
      for (int i = 0; i < values.length; i++)
      {
         mapToPack.put(trajectoryPlayer.getKeyMatrix().get(i), values[i]);
      }
   }

   public boolean isPaused(){
      return paused;
   }
}
