package us.ihmc.darpaRoboticsChallenge.configuration;

public enum LocalCloudMachines
{
   CLOUDMINION_1("cloudminion-1", "192.168.6.231"), 
   CLOUDMINION_2("cloudminion-2", "192.168.6.232"), 
   CLOUDMINION_3("cloudminion-3", "192.168.6.233"), 
   CLOUDMINION_4("cloudminion-4", "192.168.6.234"), 
   CLOUDMINION_5("cloudminion-5", "192.168.6.235"), 
   CLOUDMINION_6("cloudminion-6", "192.168.6.236"), 
   CLOUDMINION_7("cloudminion-7", "192.168.6.237"), 
   CLOUDMINION_8("cloudminion-8", "192.168.6.238"), 
   LOCALHOST("localhost", "localhost");

   private final String host;
   private final String ip;

   private LocalCloudMachines(String host, String ip)
   {
      this.host = host;
      this.ip = ip;
   }

   public String getHost()
   {
      return host;
   }

   public String getIp()
   {
      return ip;
   }

   @Override
   public String toString()
   {
      return getHost() + " (" + getIp() + ")";
   }

   public String toShortString()
   {
      return toString().replace("cloud", "").replace("-", " ");
   }
}