package us.ihmc.darpaRoboticsChallenge.maxwellPro;


public class MaxwellTime
{
   private int hours = 0;
   private int minutes = 0;
   private int seconds = 0;
   
   private String stringRepresentation = "0:00:00";
   
   public MaxwellTime()
   {
      this("0:00:00");
   }
   
   public MaxwellTime(String time)
   {
      checkTimeFormat(time);
   }
   
   private void checkTimeFormat(String time)
   {
      String[] hoursMinutesSecondsString = time.split(":", 3);
      if(hoursMinutesSecondsString.length != 3)
         throw new RuntimeException("Invalid time format!");
      
      int hours = Integer.valueOf(hoursMinutesSecondsString[0]);
      int minutes = Integer.valueOf(hoursMinutesSecondsString[1]);
      int seconds = Integer.valueOf(hoursMinutesSecondsString[2]);
      
      if(!(hours < 0 || minutes < 0 || seconds < 0))
      {
         this.hours = hours;
         this.minutes = minutes;
         this.seconds = seconds;
         
         updateTime();
      }
      else
         throw new RuntimeException("Invalid time format: contains negative number");
   }
   
   private void updateTime()
   {
      while((seconds / 60) >= 1)
      {
         minutes++;
         seconds -= 60;
      }
      
      while((minutes / 60) >= 1)
      {
         hours++;
         minutes -= 60;
      }
      
      hours %= 10;
      
      String hoursString = String.valueOf(hours);
      String minutesString = "";
      String secondsString = "";
      
      if(minutes < 10)
         minutesString += "0";
      minutesString += String.valueOf(minutes);
      
      if(seconds < 10)
         secondsString += "0";
      secondsString += String.valueOf(seconds);
      
      stringRepresentation = hoursString + ":" + minutesString + ":" + secondsString;
   }
   
   public void addSeconds(long seconds)
   {
      this.seconds += seconds;
      updateTime();
   }
   
   public long inSeconds()
   {
      return (3600 * hours) + (60 * minutes) + seconds;
   }
   
   @Override
   public String toString()
   {
      return stringRepresentation;
   }
   
   @Override
   public boolean equals(Object obj)
   {
      if(obj instanceof MaxwellTime)
         if(compareTo((MaxwellTime)obj) == 0)
            return true;
      
      return false;
   }
   
   public boolean isGreaterThan(MaxwellTime time)
   {
      return this.compareTo(time) == 1;
   }
   
   public boolean isLessThan(MaxwellTime time)
   {
      return this.compareTo(time) == -1;
   }
   
   private int compareTo(MaxwellTime time)
   {
      this.updateTime();
      time.updateTime();
      
      int hoursDiff = this.hours - time.hours;
      int minutesDiff = this.minutes - time.minutes;
      int secondsDiff = this.seconds - time.seconds;
      
      if(hoursDiff >= 0)
      {
         if(hoursDiff == 0)
         {
            if(minutesDiff >= 0)
            {
               if(minutesDiff == 0)
               {
                  if(secondsDiff >= 0)
                  {
                     if(secondsDiff == 0)
                        return 0;
                     else
                        return 1;
                  }
                  else
                     return -1;
               }
               else
                  return 1;
            }
            else
               return -1;
         }
         else
            return 1;
      }
      else
         return -1;
   }
}
