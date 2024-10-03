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

public class ReplaySyncedCSV
{
   private TrajectoryRecordReplay trajectoryPlayer;

   private Long startTime;
   private Double startCSVTime;

   private boolean paused = false;
   private boolean finished = false;

   private HashMap<String, Double> currentFrame = new HashMap<>();
   private HashMap<String, Double> nextFrame = new HashMap<>();

   public ReplaySyncedCSV(String filePath, int numberOfParts)
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
         makeMap(trajectoryPlayer.play(true), currentFrame);
         makeMap(trajectoryPlayer.play(true), nextFrame);
         startCSVTime = currentFrame.get("time[sec]");
         finished = false;
         LogTools.info("all keys: " + trajectoryPlayer.getKeyMatrix());
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
      paused = true;
   }

   public void stop()
   {
      reset();
      finished = true;
   }

   public boolean checkNewFrame(Long currentTime)
   {
      if (paused || finished)
      {
         return false;
      }

      if (startTime == null)
      {
         if (!Objects.equals(startCSVTime, currentFrame.get("time[sec]")))
         {
            startTime = (long) (currentTime - currentFrame.get("time[sec]") * 1_000_000_000);
         }
         else
         {
            startTime = currentTime;
         }
      }
      if (trajectoryPlayer.hasDoneReplay()){
         LogTools.info("Finished replay111");
         stop();
      }
      if ((currentTime - startTime) / 1_000_000_000.0 < nextFrame.get("time[sec]") - startCSVTime)
      {
         LogTools.info("No new frame");
         return false;
      }

      while ((currentTime - startTime) / 1_000_000_000.0 >= nextFrame.get("time[sec]") - startCSVTime)
      {
         currentFrame.putAll(nextFrame);
         makeMap(trajectoryPlayer.play(true), nextFrame);
      }
      LogTools.info("NextFrame Time: " + nextFrame.get("time[sec]") + " Current Time: " + (currentTime - startTime) / 1_000_000_000.0);

      return true;
   }
   public HashMap<String, Double> getCurrentFrame()
   {
      return currentFrame;
   }

   private void reset()
   {
      trajectoryPlayer.reset();
      currentFrame.clear();
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
