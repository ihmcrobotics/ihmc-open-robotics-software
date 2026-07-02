package us.ihmc.tools.logic;

import us.ihmc.yoVariables.providers.BooleanProvider;

import java.util.ArrayList;

/**
 * Simple tool to assemble an ordered sequence of runnable logical elements that must be
 * executed in a specific order. Each logical element in the list has a corresponding
 * prerequisite logical condition that must be true in order to begin the logic in the
 * given element, and a completion logical condition that must be true in order to move
 * on to the next element in the list
 *
 * @author Stefan Fasano
 */
public class OrderedLogicalSequence
{
   private final ArrayList<Runnable> logicalElements = new ArrayList<>();
   private final ArrayList<BooleanProvider> prerequisiteConditions = new ArrayList<>();
   private final ArrayList<BooleanProvider> completionConditions = new ArrayList<>();
   private final ArrayList<Boolean> allowRepetitiveExecution = new ArrayList<>();

   private int currentLogicalElement = 0;
   private int lastLogicalElementRun = -1;
   private boolean started = false;
   private boolean finished = false;

   public OrderedLogicalSequence()
   {
   }

   /**
    * Adds a logical element to this ordered logical sequence
    * @param runnable               The main logic to run
    * @param preRequisiteCondition  Prerequisite logic that must be true before running main logic
    * @param completionCondition    Completion logic that must be true before moving to the next element
    */
   public void addLogicalElement(Runnable runnable, BooleanProvider preRequisiteCondition, BooleanProvider completionCondition)
   {
      addLogicalElement(runnable, preRequisiteCondition, completionCondition, false);
   }

   /**
    * Adds a logical element to this ordered logical sequence
    * @param runnable                  The main logic to run
    * @param prerequisiteCondition     Prerequisite logic that must be true before running main logic
    * @param completionCondition       Completion logic that must be true before moving to the next element
    * @param allowRepetitiveExecution  If true, main logic will run every update tick until completion condition is met
    */
   public void addLogicalElement(Runnable runnable, BooleanProvider prerequisiteCondition, BooleanProvider completionCondition, boolean allowRepetitiveExecution)
   {
      if (allowRepetitiveExecution && completionCondition == null)
         throw new IllegalArgumentException("Cannot add a logical element with repetitive execution and no completion condition");
      logicalElements.add(runnable);
      prerequisiteConditions.add(prerequisiteCondition);
      completionConditions.add(completionCondition);
      this.allowRepetitiveExecution.add(allowRepetitiveExecution);
   }

   /**
    * Calling this will allow execution of this ordered logical sequence to begin
    */
   public void start()
   {
      started = true;
   }

   /**
    * Resets this ordered logical sequence so it can be used again
    */
   public void reset()
   {
      started = false;
      finished = false;
      currentLogicalElement = 0;
      lastLogicalElementRun = -1;
   }

   /**
    * Main update loop of this ordered logical sequence
    */
   public void update()
   {
      // If the sequence has not been started, or if it is finished, skip this update
      if (!started || finished)
         return;

      // If the completion condition has been met for this element, increment to the next element
      // This gets checked first to avoid unnecessary run of logical element if completion is already met
      if (checkCompletionCondition())
      {
         incrementLogicalElement();
         return;
      }

      // If the prerequisite condition has been met for this element, run the runnable logic of this element
      if (checkPrerequisiteCondition())
      {
         if (allowRepetitiveExecution.get(currentLogicalElement))
         {
            logicalElements.get(currentLogicalElement).run();
            lastLogicalElementRun = currentLogicalElement;
            if (checkCompletionCondition())
               incrementLogicalElement();
         }
         else if (currentLogicalElement != lastLogicalElementRun)
         {
            logicalElements.get(currentLogicalElement).run();
            lastLogicalElementRun = currentLogicalElement;
            if (completionConditions.get(currentLogicalElement) == null || checkCompletionCondition())
               incrementLogicalElement();
         }
      }
   }

   /**
    * Checks prerequisite condition for current logical element
    * @return true if prerequisite condition is met, false otherwise
    */
   private boolean checkPrerequisiteCondition()
   {
      return prerequisiteConditions.get(currentLogicalElement) == null || prerequisiteConditions.get(currentLogicalElement).getValue();
   }

   /**
    * Checks completion condition for current logical element
    * @return true if completion condition is met, false otherwise
    */
   private boolean checkCompletionCondition()
   {
      return completionConditions.get(currentLogicalElement) != null && completionConditions.get(currentLogicalElement).getValue();
   }

   /**
    * Increments logical element and checks if sequence is finished
    */
   private void incrementLogicalElement()
   {
      currentLogicalElement++;
      if (currentLogicalElement >= logicalElements.size())
         finished = true;
   }

   /**
    * @return {@link OrderedLogicalSequence#started}: if the logical sequence has started
    */
   public boolean hasStarted()
   {
      return started;
   }

   /**
    * @return {@link OrderedLogicalSequence#finished}: if the logical sequence has finished
    */
   public boolean hasFinished()
   {
      return finished;
   }

   /**
    * @return {@link OrderedLogicalSequence#currentLogicalElement}: index of the current logical element
    */
   public int getCurrentLogicalElement()
   {
      return currentLogicalElement;
   }
}
